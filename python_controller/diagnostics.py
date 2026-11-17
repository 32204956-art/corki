"""Passive network checks and evidence-based reports. Never creates motion packets."""
from datetime import datetime
import ipaddress
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import threading


TELEMETRY_KEYS={'type','boot_id','session_id','last_seq','seq','uptime_ms','mode','state','status','fault','cmd_vx','cmd_vy','cmd_w',
    'command_age_ms','wheel_target','wheel_speed','wheel_pwm','encoder_count','counts','rpm','control_overruns','wifi_rssi',
    'motion_id','goal_target_cm','goal_progress_cm','overshoot_cm','odom_dx_cm','odom_dy_cm','odom_dtheta_rad'}


def safe_payload(value):
    def clean(v):
        if v is None or isinstance(v,bool):return v
        if isinstance(v,int):return v if abs(v)<=2**64 else None
        if isinstance(v,float):return v if math.isfinite(v) else None
        if isinstance(v,str):return v[:120]
        if isinstance(v,list):return [clean(x) for x in v[:12]]
        return None
    return {k:clean(v) for k,v in value.items() if k in TELEMETRY_KEYS} if isinstance(value,dict) else None


def diagnostic_report(host,transport,network=None,attempt=None,trial=None):
    t=transport or {};network=network or {};observed=t.get('last_rx_json') or t.get('telemetry') or {}
    trusted=t.get('last_rx_trusted',bool(t.get('telemetry')))
    steps=[];findings=[]
    def step(name,status,message):steps.append(dict(name=name,status=status,message=message))
    def finding(code,message,evidence,certainty='HYPOTHESIS'):
        findings.append(dict(code=code,message=message,evidence=evidence,certainty=certainty))
    try:
        ip=ipaddress.IPv4Address(host)
        if ip.is_unspecified or ip.is_multicast or str(ip)=='255.255.255.255':raise ValueError()
        step('IP_ADDRESS','OK','숫자 IPv4 형식이 정상입니다. 실제 ESP32의 현재 주소인지는 별도 확인해야 합니다.')
    except ValueError:step('IP_ADDRESS','FAIL','ESP32의 현재 숫자 IPv4 주소를 입력하세요. 과거 주소를 현재 주소로 가정하지 않습니다.')
    if t.get('rx_bound'):step('RX_SOCKET','OK',f'노트북 UDP 8889 수신 소켓이 준비되었습니다. PID={os.getpid()}')
    elif attempt and attempt.get('status')=='FAILED' and attempt.get('phase')!='CONFIG':
        code=attempt.get('error_code')
        step('RX_SOCKET','FAIL',f'UDP 8889 수신 소켓 준비 실패: {"다른 프로그램의 포트 점유 가능성" if code in (10048,98) else "소켓 오류 확인 필요"} (코드 {code})')
    else:step('RX_SOCKET','NOT_TESTED','아직 실제 UDP 연결을 시작하지 않았습니다. IP 저장만으로 소켓이 열리지 않습니다.')
    if t.get('rx_packets',0) or t.get('telemetry'):
        step('ESP32_RESPONSE','OK' if trusted else 'FAIL','UDP 데이터가 도착했습니다.' if trusted else '설정한 주소/포트와 다른 송신자의 데이터입니다. 제어에 사용하지 않습니다.')
    elif t.get('rx_bound'):step('ESP32_RESPONSE','PENDING','텔레메트리를 기다립니다. 현재 IP·동일 네트워크·방화벽·공유기 단말 격리를 확인하세요.')
    else:step('ESP32_RESPONSE','NOT_TESTED','수신 소켓 준비 후 ESP32 응답을 확인합니다.')
    detail=t.get('validation_detail')
    if detail in ('SESSION_MISMATCH','SEQUENCE_NOT_SENT','SEQUENCE_TOO_OLD','REPLAYED_OR_OUT_OF_ORDER'):
        step('SESSION_HEARTBEAT','FAIL',f'수신 세션/명령 순서 확인에 실패했습니다 ({detail}). 다른 제어 프로그램이나 업로드된 펌웨어를 확인하세요.')
    elif t.get('stop_verified'):
        heartbeat=t.get('heartbeat_verified',False)
        step('SESSION_HEARTBEAT','OK','새 STOP의 세션·순서·정지 응답이 확인되었습니다. '+('생존 확인 응답도 확인되었습니다.' if heartbeat else '생존 확인 응답은 아직 미검증입니다.'))
    else:step('SESSION_HEARTBEAT','PENDING' if t.get('rx_bound') else 'NOT_TESTED','새 STOP 이후의 유효 세션·순서·정지 응답을 확인해야 합니다.')
    if trusted and observed.get('mode') and observed['mode']!='NETWORK':
        step('ESP32_CONTROL','FAIL',f'수신 원문의 제어 모드가 NETWORK가 아닙니다 ({observed["mode"]}). 시리얼에서 STOP, AUTO, STATUS 순서로 확인하세요.')
        finding('MODE_NOT_NETWORK','수신 원문에서 네트워크 제어 모드가 아님을 관측했습니다.',{'mode':observed['mode']},'OBSERVED')
    elif trusted and observed.get('fault') not in (None,'NONE'):
        step('ESP32_CONTROL','FAIL',f'ESP32가 오류를 보고했습니다 ({observed["fault"]}). 원인을 제거하고 명시적으로 복구해야 합니다.')
    elif t.get('telemetry_valid'):step('ESP32_CONTROL','OK','유효한 NETWORK 텔레메트리이며 현재 보고된 오류가 없습니다.')
    else:step('ESP32_CONTROL','NOT_TESTED','현재 ESP32의 모드와 오류 상태를 유효하게 확인하지 못했습니다.')
    complete=all(isinstance(observed.get(k),list) and len(observed[k])==3 for k in ('wheel_target','wheel_speed','wheel_pwm','encoder_count'))
    if t.get('stop_verified') and t.get('telemetry_valid') and complete:
        step('MOTOR_READY','OK','통신상 준비 조건을 충족했습니다. 바퀴 공중 고정·모터 전원·물리 정지 수단 확인과 별도 사용자 승인이 여전히 필요합니다.')
    else:step('MOTOR_READY','NOT_TESTED','정지·오류·휠/엔코더 수신 조건이 확인되기 전에는 모터 시험을 활성화할 수 없습니다.')
    if network.get('same_subnet') is False:
        finding('DIFFERENT_SUBNET','선택된 노트북 인터페이스와 입력 주소가 같은 직접 연결 대역이 아닙니다. 라우팅이 가능한지는 별도 확인해야 합니다.',
                {'local_ip':network.get('selected_local_ip'),'target':host},'OBSERVED')
    if network.get('ping_reply') is False:
        finding('NO_ICMP_REPLY','ICMP 응답이 없습니다. 주소 오류·전원·격리·ICMP 차단 등 후보를 구분해야 하며 이것만으로 UDP 실패를 확정하지 않습니다.',{'ping_reply':False},'OBSERVED')
    if not t.get('last_tx'):
        finding('NO_TX_OBSERVED','실제 송신 기록이 없습니다. 연결 시작·승인·소켓 준비 단계부터 확인해야 합니다.',{'last_tx':None},'OBSERVED')
    elif t['last_tx'].get('type')!='cmd_vel' and not trial:
        finding('NO_MOTION_TRIAL','현재 기록은 무구동 통신 또는 정지입니다. 모터 이동이 실행됐다는 증거는 없습니다.',{'type':t['last_tx'].get('type')},'OBSERVED')
    if trial:
        if trial.get('first_motion_seq','UNKNOWN') is None:
            finding('MOTION_NOT_SENT','시험의 첫 이동 송신 순서가 없습니다. 활성화 조건과 정지·승인 상태를 먼저 확인하세요.',{'first_motion_seq':None},'OBSERVED')
        if not trial.get('command_processed'):
            finding('NO_COMMAND_PROCESSING_EVIDENCE','명령 처리 일치가 미확인입니다. 송신 순서·상태·속도 원문을 먼저 비교하세요.',{'command_processed':False})
        samples=trial.get('samples',[])
        for m in trial.get('motors',[]):
            if not trial.get('command_processed') or m.get('encoder_evidence')!='NO_ENCODER_CHANGE_OBSERVED':continue
            index=m['motor']-1
            pwms=[s['wheel_pwm'][index] for s in samples if isinstance(s.get('wheel_pwm'),list) and len(s['wheel_pwm'])==3]
            if not pwms and m.get('peak_pwm') is not None:pwms=[m['peak_pwm']]
            if not pwms:continue
            pwm_nonzero=any(isinstance(v,(int,float)) and v!=0 for v in pwms)
            finding('PWM_WITHOUT_ENCODER_CHANGE' if pwm_nonzero else 'ZERO_PWM_AND_NO_ENCODER',
                f'M{m["motor"]}: '+('PWM은 관측됐지만 엔코더 변화가 없습니다. 전원·드라이버·배선·모터·엔코더를 실제 측정으로 확인하세요. 낮은 출력의 정지 마찰도 가능성이며 제한을 자동으로 높이지 않습니다.' if pwm_nonzero else
                '명령 처리 정황은 있으나 PWM·엔코더가 0입니다. 제어 단계·출력 차단·오류를 확인해야 합니다.'),{'motor':m['motor'],'pwm_observed':pwms})
        if any(m.get('encoder_evidence')=='ENCODER_SIGN_DISAGREES' for m in trial.get('motors',[])):
            finding('ENCODER_DIRECTION_MISMATCH','기대 부호와 엔코더 변화가 다릅니다. 모터/엔코더 배선과 방향 설정을 실물로 확인하세요.',trial.get('motors'))
    if (t.get('last_tx') or {}).get('type')=='cmd_vel' and (t.get('last_tx_interval_s') or 0)>.3:
        finding('TX_INTERVAL_EXCEEDS_WATCHDOG','최근 송신 간격이 300ms를 초과했습니다. ESP32의 실제 워치독 정지 여부는 수신 오류와 함께 확인하세요.',{'interval_s':t['last_tx_interval_s']},'OBSERVED')
    return dict(generated_at=datetime.now().astimezone().isoformat(),target=host,steps=steps,findings=findings,
        network=network,connection_attempt=attempt,transport=t,trial=trial,physical_motor_result='NOT TESTED',
        hardware_checks=['시리얼 115200bps에서 현재 IP와 STATUS 확인','모터 전원 전압·공통 GND·드라이버 STBY 실측',
                         '모터/엔코더 핀과 풀업 확인','물리 전원 차단 수단과 바퀴 공중 고정 확인'],
        note='명령 처리 정황·엔코더 관측·실물 회전 검증은 별개입니다. 원인이 불명확하면 하드웨어 원인을 확정하지 않습니다.')


def inspect_network(host,rx_owned=False):
    ipaddress.IPv4Address(host)
    result=dict(checked_at=datetime.now().astimezone().isoformat(),target=host,motor_packets_sent=0,
                selected_local_ip=None,same_subnet=None,interfaces=[],ping_reply=None,port_available=None)
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
            sock.connect((host,8888))  # Route selection only; no UDP payload is sent.
            result['selected_local_ip']=sock.getsockname()[0]
    except OSError as exc:result['route_error_code']=getattr(exc,'winerror',None) or exc.errno
    if rx_owned:result['port_available']='OWN_RECEIVER_BOUND'
    else:
        try:
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:sock.bind(('',8889))
            result['port_available']=True
        except OSError as exc:
            result.update(port_available=False,bind_error_code=getattr(exc,'winerror',None) or exc.errno)
    if os.name=='nt':
        command="""[Console]::OutputEncoding=[System.Text.Encoding]::UTF8
@{interfaces=@(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | Select-Object InterfaceAlias,IPAddress,PrefixLength)
firewall_profiles=@(Get-NetFirewallProfile -ErrorAction SilentlyContinue | Select-Object Name,@{n='Enabled';e={$_.Enabled.ToString()}},@{n='DefaultInboundAction';e={$_.DefaultInboundAction.ToString()}},@{n='DefaultOutboundAction';e={$_.DefaultOutboundAction.ToString()}})
connection_profiles=@(Get-NetConnectionProfile -ErrorAction SilentlyContinue | Select-Object InterfaceAlias,@{n='NetworkCategory';e={$_.NetworkCategory.ToString()}})
udp_receivers=@(Get-NetUDPEndpoint -LocalPort 8889 -ErrorAction SilentlyContinue | Select-Object LocalAddress,LocalPort,OwningProcess)} | ConvertTo-Json -Depth 4 -Compress"""
        try:
            p=subprocess.run(['powershell','-NoProfile','-NonInteractive','-Command',command],capture_output=True,timeout=12,creationflags=0x08000000)
            data=json.loads(p.stdout.decode('utf-8-sig',errors='replace'))
            result.update(data)
            selected=next((d for d in result['interfaces'] if d.get('IPAddress')==result['selected_local_ip']),None)
            if selected:
                result['selected_interface']=selected['InterfaceAlias']
                result['same_subnet']=ipaddress.IPv4Address(host) in ipaddress.IPv4Network(f'{selected["IPAddress"]}/{selected["PrefixLength"]}',strict=False)
            p=subprocess.run(['ping','-n','1','-w','1200',host],capture_output=True,timeout=4,creationflags=0x08000000)
            result['ping_reply']=p.returncode==0
        except (OSError,ValueError,subprocess.TimeoutExpired) as exc:result['inspection_error']=type(exc).__name__
    return result


class NetworkProbe:
    def __init__(self):self.result=None;self.error=None;self.running=False
    def start(self,host,rx_owned=False):
        if self.running:return
        ipaddress.IPv4Address(host)
        self.running=True;self.result=None;self.error=None
        def run():
            try:self.result=inspect_network(host,rx_owned)
            except Exception as exc:self.error=type(exc).__name__
            finally:self.running=False
        threading.Thread(target=run,name='passive-network-diagnostics',daemon=True).start()


def save_report(folder,report):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    name=datetime.now().astimezone().strftime('diagnosis_%Y%m%d_%H%M%S_%f')
    path=folder/(name+'.json');path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    lines=['# 연결 및 모터 진단 보고서',f'\n시각: {report["generated_at"]}',f'\n대상: {report["target"]}', '\n## 단계별 결과']
    states={'OK':'정상','FAIL':'실패','PENDING':'확인 중','NOT_TESTED':'미검증'}
    names=['IP 설정','UDP 수신 대기','ESP32 응답','세션 및 생존 통신','ESP32 제어 상태','모터 시험 준비']
    for name,s in zip(names,report['steps']):lines.append(f'- {name}: {states[s["status"]]} — {s["message"]}')
    lines.append('\n## 근거와 원인 후보')
    for f in report['findings']:lines.append(f'- {f["message"]}\n  - 근거: {json.dumps(f["evidence"],ensure_ascii=False)}')
    lines.extend(['\n## 실행 및 송수신 원문',
        '필드명은 프로토콜 원문입니다. 시각은 보고서 생성 시각과, 실행 내 경과 비교용 단조 시계 시각을 구분합니다.',
        '```json',json.dumps({k:report.get(k) for k in ('program_mode','evidence_source','network','connection_attempt','transport','trial','operator_hardware_checks','serial_ports')},ensure_ascii=False,indent=2,allow_nan=False),'```'])
    lines.extend(['\n## 실물 확인 필요',*['- '+s for s in report['hardware_checks']],'\n실제 모터 검증: 미검증. 모터 자동 구동은 수행하지 않았습니다.'])
    path.with_suffix('.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    return path
