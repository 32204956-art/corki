"""Explicit, lifted-wheel bench tests; camera/gesture commands cannot drive this client."""
from collections import deque
import copy
from dataclasses import asdict
import math
import socket
import time
import ipaddress
import json
from pathlib import Path

from python_controller.communication.esp32_udp import UdpClient,number
from python_controller.models import MotionCommand
from python_controller.monitoring import vector


def connection_host(host):
    address=ipaddress.IPv4Address(host)
    if address.is_unspecified or address.is_multicast or str(address)=='255.255.255.255':
        raise ValueError('Use the numeric unicast IPv4 address of the ESP32')
    return str(address)


def save_connection(path,host):
    host=connection_host(host)
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps({'esp32_ipv4':host},indent=2)+'\n',encoding='utf-8')


def load_connection(path):
    try:return connection_host(json.loads(Path(path).read_text(encoding='utf-8'))['esp32_ipv4'])
    except (OSError,ValueError,KeyError,TypeError):return ''


def body_test_command(name,config):
    axes={'FORWARD':(1,0,0),'BACKWARD':(-1,0,0),'LEFT':(0,1,0),'RIGHT':(0,-1,0),
          'TURN_LEFT':(0,0,1),'TURN_RIGHT':(0,0,-1)}
    if name not in axes:raise ValueError('Unknown motor test direction')
    x,y,w=axes[name]
    return MotionCommand(x*config['linear_cm_s'],y*config['linear_cm_s'],w*config['angular_rad_s'])


def expected_wheels(command,radius):
    x,y,w=command.vx_cm_s,command.vy_cm_s,command.w_rad_s
    return [y+radius*w,-math.sqrt(3)/2*x-.5*y+radius*w,math.sqrt(3)/2*x-.5*y+radius*w]


def encoder_delta(current,baseline):
    return ((current-baseline+2**31)%2**32)-2**31


def summarize_trial(trial,config):
    record={k:copy.deepcopy(v) for k,v in trial.items() if k!='samples'}
    record['sample_count']=len(trial['samples'])
    record['motors']=[]
    baseline=trial['baseline_counts']
    samples=trial['samples']
    for i,expected in enumerate(trial['expected_wheel_cm_s']):
        counts=[s['encoder_count'][i] for s in samples if vector(s,'encoder_count',True) is not None]
        delta=encoder_delta(counts[-1],baseline[i]) if counts and baseline is not None else None
        measured=[s['wheel_speed'][i] for s in samples if vector(s,'wheel_speed') is not None]
        peak=max(measured,key=abs) if measured else None
        pwms=[s['wheel_pwm'][i] for s in samples if vector(s,'wheel_pwm') is not None]
        if not trial['command_processed'] or delta is None:classification='INSUFFICIENT_DATA'
        elif abs(expected)<config['min_wheel_speed_cm_s']:
            classification='NOT_EXPECTED_TO_ROTATE' if abs(delta)<config['min_encoder_delta'] else 'IDLE_WHEEL_ENCODER_CHANGE'
        elif abs(delta)<config['min_encoder_delta']:classification='NO_ENCODER_CHANGE_OBSERVED'
        elif delta*expected<0:classification='ENCODER_SIGN_DISAGREES'
        else:classification='ENCODER_SIGN_AGREES'
        record['motors'].append(dict(motor=i+1,expected_wheel_cm_s=expected,encoder_delta=delta,
            peak_measured_cm_s=peak,peak_pwm=max(pwms,key=abs) if pwms else None,encoder_evidence=classification))
    return record


class MotorTestClient(UdpClient):
    requires_verified_signs=False
    manual_test_only=True

    def __init__(self,config,*,clock=time.monotonic,socket_factory=None,evidence_source='LIVE_UDP'):
        config=copy.deepcopy(config)
        config['communication']['host']=connection_host(config['communication']['host'])
        config['communication']['send_period_s']=config['motor_test']['send_period_s']
        config['safety']['telemetry_timeout_s']=config['motor_test']['telemetry_timeout_s']
        super().__init__(config,clock=clock,socket_factory=socket_factory)
        self.monitor_heartbeat=True
        self.operator_at=-math.inf
        self.trial=None
        self.history=deque(maxlen=25)
        self.events=deque()
        self.evidence_source=evidence_source
        self.trial_number=0

    def operator_alive(self):
        with self.lock:self.operator_at=self.clock()

    def submit(self,decision,frame_timestamp):
        # Camera observations never supply either motion or the operator liveness lease.
        return None

    def arm(self):
        return False  # Only the separate lifted-wheel confirmation may authorize motion.

    def _stopped(self,now):
        d=self.guard.data
        return bool(not self.failed and self.guard.ready(now) and self.stop_seq is not None and
                    d['last_seq']>=self.stop_seq and self.guard.received_at>=self.stop_requested_at)

    def probe_heartbeat(self):
        with self.lock:
            if self.trial is not None or not self._stopped(self.clock()):
                raise ValueError('Wait for fresh STOP feedback before stationary heartbeat')
            self.armed=self.active=False
            self.monitor_heartbeat=True
            self.reason='STATIONARY_HEARTBEAT'

    def authorize_manual(self,wheels_lifted,acknowledged):
        with self.lock:
            now=self.clock()
            if wheels_lifted is not True or acknowledged is not True:
                raise ValueError('Confirm lifted wheels and explicitly authorize this one trial')
            if self.trial is not None or not self._stopped(now):
                raise ValueError('Fresh matching STOPPED telemetry required; wait for the previous trial to finish')
            if any(vector(self.guard.data,k,k=='encoder_count') is None for k in ('wheel_target','wheel_speed','wheel_pwm','encoder_count')):
                raise ValueError('Complete wheel/encoder telemetry required before a motor trial')
            self.armed=True;self.active=False;self.operator_at=now
            self.reason='MANUAL_TEST_READY'

    def begin_test(self,name,duration=None):
        with self.lock:
            now=self.clock();config=self.cfg['motor_test']
            duration=config['duration_s'] if duration is None else duration
            if not number(duration) or not .1<=duration<=config['max_duration_s']:
                raise ValueError('Motor test duration is outside the permitted range')
            command=body_test_command(name,config)
            if not self.armed or self.trial is not None or not self._stopped(now):
                raise ValueError('Authorize a stopped, lifted-wheel trial first')
            if not 0<=now-self.operator_at<=config['ui_lease_s']:
                self._latch('OPERATOR_LEASE_EXPIRED');raise ValueError('Operator UI lease expired')
            self.trial_number+=1
            self.trial=dict(id=self.trial_number,direction=name,requested=asdict(command),
                session_id=self.protocol.session_id,
                expected_wheel_cm_s=expected_wheels(command,config['body_radius_cm']),
                started_at=now,deadline=now+duration,duration_s=duration,ended_at=None,completed_at=None,
                first_motion_seq=None,last_motion_seq=None,baseline_counts=list(self.guard.data['encoder_count']),
                samples=[],command_processed=False,physical_result='NOT TESTED',physical_observations=None,
                stop_verified=False,stop_reason=None,evidence_source=self.evidence_source)
            self.command=command;self.active=True;self.monitor_heartbeat=False;self.reason='MANUAL_TEST_RUNNING'
            self.events.append(dict(event='MOTOR_TEST_STARTED',trial=summarize_trial(self.trial,config)))

    def _latch(self,reason):
        if self.trial is not None and self.trial['ended_at'] is None:
            self.trial.update(ended_at=self.clock(),stop_reason=reason)
            self.events.append(dict(event='MOTOR_TEST_STOP_REQUESTED',trial=summarize_trial(self.trial,self.cfg['motor_test'])))
        self.monitor_heartbeat=False
        super()._latch(reason)

    def _on_telemetry(self,data,now):
        if self.trial is None:return
        t=self.trial
        sample={k:copy.deepcopy(data.get(k)) for k in ('last_seq','uptime_ms','state','fault','cmd_vx','cmd_vy','cmd_w','wheel_target','wheel_speed','wheel_pwm','encoder_count')}
        sample['received_at']=now
        if len(t['samples'])<64:t['samples'].append(sample)
        matched=next((p for p in reversed(self.tx_history) if p['seq']==data['last_seq']),None)
        if (matched and matched['type']=='cmd_vel' and t['first_motion_seq'] is not None and
            t['first_motion_seq']<=data['last_seq']<=t['last_motion_seq'] and data['state']=='VELOCITY' and data['fault']=='NONE' and
            all(abs(data[field]-matched[wire])<.001 for field,wire in (('cmd_vx','vx'),('cmd_vy','vy'),('cmd_w','w')))):
            t['command_processed']=True

    def _after_send(self,packet):
        self.tx_history[-1]['operator_lease_at']=self.operator_at if math.isfinite(self.operator_at) else None
        if self.trial is not None and packet['type']=='cmd_vel':
            if self.trial['first_motion_seq'] is None:self.trial['first_motion_seq']=packet['seq']
            self.trial['last_motion_seq']=packet['seq']

    def _finish(self,verified):
        self.trial.update(completed_at=self.clock(),stop_verified=verified)
        record=summarize_trial(self.trial,self.cfg['motor_test'])
        self.history.append(record)
        self.events.append(dict(event='MOTOR_TEST_COMPLETED',trial={**record,'samples':copy.deepcopy(self.trial['samples'])}))
        self.trial=None

    def _select_packet(self,now):
        config=self.cfg['motor_test']
        if self.armed and not self.guard.healthy(now):self._latch('TELEMETRY_STALE_OR_INVALID')
        if self.armed and not 0<=now-self.operator_at<=config['ui_lease_s']:self._latch('OPERATOR_LEASE_EXPIRED')
        if self.active and self.trial is not None and now>=self.trial['deadline']:self._latch('TEST_DURATION_COMPLETE_REARM_REQUIRED')
        if self.trial is not None and self.trial['ended_at'] is not None:
            if self._stopped(now):self._finish(True)
            elif now-self.trial['ended_at']>=config['stop_feedback_timeout_s']:self._finish(False)
        if self.armed and self.active:
            return self.protocol.velocity(self.command),True
        if self.armed and not self.guard.ready(now):self._latch('UNEXPECTED_IDLE_MOTION_OR_FAULT')
        if self.monitor_heartbeat and self._stopped(now):
            self.reason='MANUAL_TEST_READY' if self.armed else 'STATIONARY_HEARTBEAT'
            return self.protocol.heartbeat(),False
        if self.monitor_heartbeat and self.guard.data is not None and self.guard.data['state']!='STOPPED':
            self._latch('UNEXPECTED_IDLE_MOTION_OR_FAULT')
        return self.protocol.stop(),False

    def snapshot(self):
        with self.lock:
            data=super().snapshot()
            status='RUNNING' if self.active else ('WAITING_STOP_FEEDBACK' if self.trial else ('READY' if self.armed else 'DISABLED'))
            data.update(manual_test_only=True,manual_enabled=self.armed,motor_test=dict(status=status,
                command=asdict(self.command) if self.active else asdict(MotionCommand()),
                remaining_s=max(0,self.trial['deadline']-self.clock()) if self.active else 0,
                trial=summarize_trial(self.trial,self.cfg['motor_test']) if self.trial else None,
                history=copy.deepcopy(list(self.history)),evidence_source=self.evidence_source))
            return data

    def drain_events(self):
        with self.lock:
            events=list(self.events);self.events.clear();return events

    def record_observations(self,trial_id,observations,notes):
        with self.lock:
            allowed={'NOT TESTED','MATCHES EXPECTED','OPPOSITE','NO ROTATION'}
            if len(observations)!=3 or any(v not in allowed for v in observations) or not notes.strip():
                raise ValueError('Record all three observations and nonempty notes')
            trial=next((t for t in self.history if t['id']==trial_id),None)
            if trial is None:raise ValueError('Select a completed trial')
            trial.update(physical_result='OPERATOR_RECORDED',physical_observations=list(observations),operator_notes=notes)
            self.events.append(dict(event='MOTOR_TEST_OPERATOR_OBSERVATION',trial=copy.deepcopy(trial)))

    def close(self):
        super().close()
        with self.lock:
            if self.trial is not None:self._finish(False)
