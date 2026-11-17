"""Presentation-only Korean labels. Protocol fields and stored identifiers stay intact."""
import re

TEXT={
 'Omni Robot | Validation Console':'옴니 로봇 | 실시간 검증 대시보드','OMNI / VALIDATION CONSOLE':'옴니 로봇 · 검증 대시보드',
 'DASHBOARD':'종합 현황','VISION':'영상 인식','MOTION':'이동 및 모터','TEST & LOG':'시험 및 기록','MOTOR TEST':'모터 동작 시험',
 'EMERGENCY STOP   [SPACE]':'긴급 정지 [Space]','Confirm target [Enter]':'목표 확정 [Enter]','Arm [A]':'제어 준비 [A]',
 'Resume [R]':'추종 재개 [R]','Pause [P]':'일시 정지 [P]','Clear target [C]':'목표 해제 [C]',
 'Initializing camera and models in background...':'카메라와 인식 모델을 준비하고 있습니다.','Waiting for frames':'카메라 영상을 기다립니다.',
 'Relative height target: --':'상대 크기 목표: 미설정','Waiting for camera / model initialization':'카메라 연결 및 인식 모델 초기화를 기다립니다.',
 'TARGET / TRACKING':'목표 인물 및 추적','People':'검출 인원','Target ID':'목표 추적 ID','Confirmed / Lost':'목표 확정 / 상실',
 'Confidence':'신뢰도','Center (px)':'중심 좌표 (px)','Last seen / age (s)':'마지막 검출 / 경과 (초)','Horizontal error':'수평 오차',
 'Relative height':'상대 높이 비율','01 / RAW GESTURE':'① 최초 인식 손동작','02 / CONFIRMED GESTURE':'② 확정된 손동작','03 / APPLIED ACTION':'③ 적용된 동작',
 'Hands':'검출된 손 수','Raw labels / confidence / owner':'최초 분류 / 신뢰도 / 연결 목표','Sample age (s)':'관측 경과 (초)',
 'Current confirmation':'현재 확정 손동작','Time / required (s)':'연속 시간 / 기준 (초)','Frames / required':'연속 프레임 / 기준',
 'Confidence threshold':'신뢰도 기준','Applied this update':'이번 처리에 적용된 동작','Last gesture action / age (s)':'마지막 적용 / 경과 (초)',
 'Rejection / hold reason':'거부 또는 유지 사유','CONTROL / SAFETY':'이동 제어 및 안전','State':'현재 상태','Requested vx / vy / w':'요청 속도 vx / vy / w',
 'Safe vx / vy / w':'안전 적용 속도 vx / vy / w','Limits or stop applied':'제한 또는 정지 적용','Final stop / status':'최종 정지 또는 상태 사유','Radar':'후방 레이더',
 'ESP32 / TRANSPORT':'ESP32 연결 및 송수신','Connection':'연결 상태','Receive validity / reason':'수신 유효성 / 사유','Last RX / age (s)':'최근 수신 / 경과 (초)',
 'Session / telemetry last_seq':'세션 / 마지막 처리 seq','Last successful UDP send':'최근 실제 UDP 송신','Telemetry seq match (not ACK)':'텔레메트리 seq 일치 (별도 응답 아님)',
 'ESP32 state / fault':'ESP32 상태 / 오류','ESP32 reported desired cmd':'ESP32 보고 희망 명령','Post-limit body command':'제한 이후 본체 적용 명령',
 'Stop transmission / feedback':'정지 송신 / 수신 확인','Motor':'모터','Target cm/s':'목표 속도 (cm/s)','Measured cm/s':'측정 속도 (cm/s)',
 'Target RPM*':'환산 목표 RPM*','Measured RPM*':'환산 측정 RPM*','PWM / encoder':'PWM / 엔코더',
 '* RPM converted using configured wheel radius. Encoders do not prove floor motion.':'* RPM은 설정된 바퀴 반지름으로 환산합니다. 엔코더 회전만으로 본체의 지상 이동을 검증하지 않습니다.',
 'ESP32 CONNECTION / STOP proof then stationary heartbeat':'ESP32 연결 · 정지 증명 후 무구동 생존 확인',
 'ESP32 IPv4 address':'ESP32의 현재 IPv4 주소','Authorize UDP connection only (does not authorize motor motion)':'UDP 통신 연결만 승인합니다 (모터 구동 승인이 아닙니다)',
 'Connect UDP':'UDP 연결','Save IP':'IP 저장','Stop + return to SIMULATION':'정지 후 시뮬레이션 복귀',
 'TX 8888 / RX 8889. No separate ACK packet. Connection sends no cmd_vel. Automatic hardware following is disabled.':
 '명령 송신 8888 / 텔레메트리 수신 8889. 별도 ACK 패킷은 없습니다. 연결만으로 이동 명령을 보내지 않으며 실제 장치 자동 추종은 차단됩니다.',
 'Logging disabled (test)':'기록 비활성화 (자동 검사)','JSONL session log (10 Hz + operator events; no camera images):':'실행별 JSONL 기록 (최대 10Hz 상태 및 조작 사건, 영상 저장 없음):',
 'Validation step':'검증 단계','Operator result':'사용자 기록 결과','Evidence / notes':'근거 및 관찰 메모',
 'Observed result / evidence (required for PASS or FAIL)':'실제 관찰 및 근거를 입력하세요 (정상/실패 기록 시 필수)',
 'I performed this step':'이 단계를 실제로 수행했습니다','Record result':'시험 결과 기록',
 'MOTOR TEST / transmission is not proof of rotation':'모터 동작 시험 · 송신만으로 실제 회전을 판정하지 않습니다',
 'All wheels are lifted and the platform is secured':'바퀴 공중 고정·모터 전원·물리 전원 차단 수단을 확인했습니다',
 'Authorize ONE motor test burst':'짧은 모터 시험 한 번을 승인합니다','Enable one MOTOR TEST':'모터 시험 1회 활성화','Stationary heartbeat check':'정지 상태 생존 통신 확인',
 'Source':'데이터 출처','Status / reason':'상태 및 사유','Remaining (s)':'남은 시간 (초)','Manual command vx / vy / w':'수동 명령 vx / vy / w',
 'Command processing evidence':'명령 처리 정황','Stop feedback':'정지 응답 확인','Current / last trial':'현재 또는 마지막 시험',
 'Forward (+vx)':'전진 (+vx)','Backward (-vx)':'후진 (-vx)','Left (+vy)':'좌측 이동 (+vy)','Right (-vy)':'우측 이동 (-vy)',
 'Turn left (+w)':'좌회전 (+w)','Turn right (-w)':'우회전 (-w)','Encoder count':'엔코더 누적값','Trial encoder evidence':'시험 중 엔코더 관측',
 'Physical observations / wiring / encoder direction notes; do not infer body travel from encoder counts':'실제 회전·배선·엔코더 방향 관찰을 기록하세요. 엔코더만으로 본체 이동을 확정하지 마세요.',
 'Record visual observations for selected completed trial':'선택한 완료 시험의 육안 관찰 저장',
 'No motor trial performed. Actual hardware: NOT TESTED.':'모터 시험 기록이 없습니다. 실제 하드웨어는 미검증입니다.',
 'EMERGENCY STOP LATCHED / explicit re-arm required':'긴급 정지가 유지됩니다. 명시적 재활성화가 필요합니다.',
 'Hardware connection refused':'실제 장치 연결 거부','IP not saved':'IP 저장 실패','Motor test not enabled':'모터 시험 활성화 실패',
 'Heartbeat not started':'생존 통신 확인 실패','Motor test not started':'모터 시험 시작 실패','Observations not recorded':'관찰 기록 저장 실패',
 'Result not recorded':'시험 결과 기록 실패','Confirm you actually performed this step.':'시험을 실제 수행했는지 먼저 확인하세요.',
 'A hardware PASS cannot be recorded from SIMULATION or synthetic preview.':'시뮬레이션 또는 합성 미리보기에서 실제 하드웨어 정상 결과를 기록할 수 없습니다.',
 'ESP32 IP saved locally; no network connection started':'ESP32 주소를 로컬에 저장했습니다. 통신 연결은 아직 시작하지 않았습니다.',
 'Command axes (proposed signs): +vx forward':'명령 축 (실물 부호 미검증): +vx 전진',
 'NO MOTOR MEASUREMENTS RECEIVED\nTarget and measured RPM history appears after valid ESP32 telemetry.':'모터 측정값을 아직 받지 못했습니다.\n유효한 텔레메트리 수신 후 목표·측정 RPM 이력이 표시됩니다.',
 'NOT REPORTED BY CURRENT FIRMWARE':'현재 펌웨어에서 보고하지 않는 값입니다',
 'Camera':'카메라','Target tracking':'목표 인물 추적','Four gestures':'네 가지 손동작','Follow command directions':'추종 명령 방향',
 'ESP32 communication':'ESP32 통신','Lifted-wheel motors':'바퀴 공중 고정 모터 시험','Controlled floor drive':'별도 승인된 지상 주행',
 'Select a target, confirm, arm, then resume.':'목표 선택 → 확정 → 제어 준비 → 추종 재개 순서로 조작하세요.',
 'SIMULATION: select a target, confirm, arm, then resume':'시뮬레이션: 목표 선택·확정 후 제어를 준비하고 재개하세요.',
 'MOTOR TEST ONLY: lift wheels and authorize one burst; A/R and gesture hardware output are disabled':'수동 모터 시험 전용: 바퀴를 띄우고 1회를 승인하세요. A/R과 손동작의 실제 장치 출력은 차단됩니다.',
 'AUTOMATIC_HARDWARE_FOLLOWING_DISABLED: use separate MOTOR TEST authorization':'실제 장치 자동 추종은 차단됩니다. 모터 동작 시험의 별도 승인을 사용하세요.',
 'ARM_DENIED: fresh, matching STOPPED telemetry required':'활성화 거부: 현재 세션과 일치하는 최신 정지 응답이 필요합니다.',
 'Confirm lifted wheels and explicitly authorize this one trial':'바퀴 공중 고정과 이번 1회 시험 승인을 확인하세요.',
 'Fresh matching STOPPED telemetry required; wait for the previous trial to finish':'현재 세션의 최신 정지 응답이 필요합니다. 이전 시험의 완료를 기다리세요.',
 'Complete wheel/encoder telemetry required before a motor trial':'시험 전에 세 모터의 목표·측정·PWM·엔코더 정보가 모두 수신되어야 합니다.',
 'Authorize a stopped, lifted-wheel trial first':'정지 응답 확인 후 바퀴 공중 고정 시험을 먼저 승인하세요.',
 'Wait for fresh STOP feedback before stationary heartbeat':'최신 정지 응답을 확인한 뒤 생존 통신을 확인하세요.',
 'Operator UI lease expired':'조작 화면의 갱신이 끊겨 구동이 차단되었습니다.',
 'Motor test duration is outside the permitted range':'시험 시간이 허용 범위를 벗어났습니다.',
 'Connect the MOTOR TEST UDP controller first':'모터 동작 시험의 UDP 연결을 먼저 시작하세요.',
 'Explicit hardware authorization required':'실제 장치 통신 연결에 대한 명시적 승인이 필요합니다.',
 'Synthetic observations cannot enable HARDWARE':'합성 입력에서는 실제 장치 연결을 허용하지 않습니다.',
 'Already connected; stop and disconnect first':'이미 연결되어 있습니다. 먼저 정지 후 연결을 해제하세요.',
 'Select a completed trial':'완료된 시험을 선택하세요.','Record all three observations and nonempty notes':'세 모터 관찰과 비어 있지 않은 메모를 기록하세요.',
 'A result requires observations/evidence in notes':'결과에는 실제 관찰과 근거 메모가 필요합니다.'
}
TEXT['LIFT ALL THREE WHEELS OFF THE FLOOR. Bench tests only. Camera/gesture motion output is disconnected.\nForward/left/rotation labels use proposed body axes; verify physical signs. One authorization allows one short burst.']='세 바퀴를 모두 바닥에서 띄우세요. 고정된 벤치 시험 전용이며 카메라·손동작 출력은 모터에 연결되지 않습니다.\n전후·좌우·회전 부호는 실물 확인이 필요합니다. 한 번의 승인으로 짧은 시험 한 번만 허용합니다.'
TEXT['1 Camera: image / latency.  2 Tracking: select, move, hide.  3 Gestures: palm, fist, up, down.\n4 Follow: left/right/near/far/lost.  5 ESP32: TX, telemetry seq/echo, feedback, disconnect.\n6 Lifted wheels: directions, encoders, RPM, STOP.  7 Floor drive: separate user approval and controlled space.\nRecording a result does not execute a motor test. All tests start NOT TESTED. No rear radar is connected.']='1 카메라: 영상·지연  2 추적: 선택·이동·가림  3 손동작: 손바닥·주먹·엄지 위/아래\n4 추종: 좌우·거리 변화·목표 상실  5 통신: 송신·텔레메트리 일치·피드백·단절\n6 바퀴 공중 고정: 방향·엔코더·RPM·정지  7 지상 주행: 별도 승인과 통제 공간 필요\n결과 기록은 시험을 실행하지 않습니다. 기본 결과는 모두 미검증이며 후방 레이더는 연결되지 않았습니다.'

MEANINGS={
 'OWN_RECEIVER_BOUND':'현재 프로그램의 수신 소켓 사용 중',
 'INVALID_IMAGE_SIZE':'잘못된 영상 크기','NOT_ARMED':'제어 준비 승인 필요','GOAL_REACHED':'목표 도달',
 'INVALID_BOOT_ID':'잘못된 부팅 식별자','INVALID_OR_REVERSE_MOTION':'허용되지 않은 방향 또는 속도',
 'INVALID_TELEMETRY_TYPE':'텔레메트리 종류 오류','PACKET_TOO_LARGE':'수신 패킷 크기 초과',
 'NOT_STARTED':'아직 시작하지 않음','RX_BIND':'수신 포트 열기','TX_SOCKET':'송신 소켓 생성','RX_BOUND':'수신 포트 준비됨',
 'CONNECTION_FAILED':'연결 실패','NO_SEPARATE_ACK_PACKET':'별도 응답 패킷 없음','MANUAL_MOTOR_TEST':'수동 모터 시험',
 'VISION_SIMULATION':'영상 기반 시뮬레이션','UDP_RX':'UDP 수신','UDP_TX':'UDP 송신',
 'SIMULATION':'시뮬레이션','HARDWARE':'실제 장치','SYNTHETIC_PREVIEW':'합성 미리보기','SYNTHETIC PREVIEW':'합성 미리보기',
 'SEARCHING':'사람 탐색 중','TARGET_SELECTED':'목표 인물 선택됨','TRACKING':'추적 중','FOLLOWING':'자동 추종 중',
 'GESTURE_OVERRIDE':'손동작 설정 적용 중','PAUSED':'일시 정지','TARGET_LOST':'목표 인물 상실','OBSTACLE_STOP':'장애물 안전 정지','FAULT':'오류 상태',
 'EMERGENCY_STOP':'긴급 정지','STARTING':'시작 중','ARMED':'활성화됨','DISARMED':'비활성화','READY':'준비됨','DISABLED':'비활성화',
 'RUNNING':'실행 중','WAITING_STOP_FEEDBACK':'정지 응답 대기','STATIONARY_HEARTBEAT':'정지 상태 생존 통신 중',
 'STARTUP_STOP':'시작 시 정지 유지','SIMULATION_NO_UDP':'시뮬레이션·UDP 전송 없음','MANUAL_TEST_READY':'수동 시험 준비됨',
 'MANUAL_TEST_RUNNING':'수동 시험 중','MANUAL_MOTOR_TEST_ONLY':'수동 모터 시험 전용·추종 출력 차단',
 'TEST_DURATION_COMPLETE_REARM_REQUIRED':'설정 시간 종료·재승인 필요','OPERATOR_LEASE_EXPIRED':'조작 화면 갱신 중단',
 'TELEMETRY_STALE_OR_INVALID':'텔레메트리 지연 또는 오류','PERCEPTION_LEASE_EXPIRED':'영상 인식 유효 시간 만료',
 'UNEXPECTED_IDLE_MOTION_OR_FAULT':'정지 중 예상 밖 이동 또는 오류','USER_STOP':'사용자 정지','USER_PAUSE':'사용자 일시 정지','CLOSED':'종료됨',
 'NO_TELEMETRY':'텔레메트리 미수신','INVALID_TELEMETRY':'잘못된 텔레메트리','OK':'정상','PASS':'정상','FAIL':'실패','PENDING':'확인 중','NOT TESTED':'미검증','NOT_TESTED':'미검증',
 'NETWORK':'네트워크 제어','MANUAL_TEST':'시리얼 수동 제어','STOPPED':'정지됨','VELOCITY':'속도 제어 중',
 'NONE':'없음','UNKNOWN':'미수신·미확인','NOT CONFIRMED':'미확정','OPERATOR_RECORDED':'사용자 육안 기록',
 'MATCHES EXPECTED':'기대 방향과 일치','OPPOSITE':'반대 방향','NO ROTATION':'회전하지 않음',
 'FORWARD':'전진','BACKWARD':'후진','LEFT':'좌측 이동','RIGHT':'우측 이동','TURN_LEFT':'좌회전','TURN_RIGHT':'우회전','STOP':'정지','SLOW':'저속',
 'Open_Palm':'손바닥 펼침','Closed_Fist':'주먹','Thumb_Up':'엄지 올림','Thumb_Down':'엄지 내림','Pointing_Up':'검지 올림','Victory':'브이 손동작','ILoveYou':'사랑 손동작',
 'RESUME':'추종 재개','PAUSE':'일시 정지','CLOSER':'목표 간격 감소','FARTHER':'목표 간격 증가',
 'NO_HAND_DETECTED':'손 미검출','TARGET_NOT_SELECTED':'목표 미선택','HAND_TARGET_MISMATCH':'손과 목표 인물 불일치',
 'MULTIPLE_PERSON_RESTRICTION':'여러 사람 검출·손동작 제어 차단','GESTURE_LOW_CONFIDENCE':'손동작 신뢰도 부족',
 'GESTURE_UNSTABLE':'손동작 안정화 중','GESTURE_NOT_MAPPED':'제어에 지정되지 않은 손동작','GESTURE_STALE':'손동작 관측 지연',
 'CONTROLLER_NOT_ARMED':'제어 준비 승인 필요','SAFETY_STOP_ACTIVE':'안전 정지 유지 중','RELEASE_RESUME_GESTURE':'손을 내린 뒤 재개 손동작을 다시 제시하세요',
 'TARGET_NOT_CONFIRMED':'목표 확정 필요','ALREADY_APPLIED_OR_ACTION_NOT_ALLOWED':'중복 적용 차단 또는 현재 상태에서 허용되지 않는 동작',
 'NO_ELIGIBLE_GESTURE':'제어 가능한 손동작 없음','NO_GESTURE_CLASSIFICATION':'손동작 분류 없음','CONFLICTING_OR_UNSUPPORTED_GESTURES':'서로 충돌하거나 미지원 손동작',
 'ENCODER_SIGN_AGREES':'엔코더 부호 일치','ENCODER_SIGN_DISAGREES':'엔코더 반대 부호','NO_ENCODER_CHANGE_OBSERVED':'엔코더 변화 미관측',
 'NOT_EXPECTED_TO_ROTATE':'회전 목표가 없는 바퀴','IDLE_WHEEL_ENCODER_CHANGE':'정지 대상 바퀴에서 엔코더 변화','INSUFFICIENT_DATA':'근거 부족',
 'MODE_NOT_NETWORK':'네트워크 제어 모드 아님','SESSION_MISMATCH':'세션 불일치','SEQUENCE_NOT_SENT':'송신하지 않은 순서 번호',
 'SEQUENCE_TOO_OLD':'오래된 순서 번호','REPLAYED_OR_OUT_OF_ORDER':'중복 또는 역순 응답','MALFORMED_JSON':'JSON 형식 오류','SOURCE_MISMATCH':'송신자 주소·포트 불일치',
 'INVALID_TELEMETRY_FIELDS':'텔레메트리 필드 또는 단위 확인 필요','INVALID_STATE_OR_FAULT':'잘못된 상태·오류 필드',
 'MOTOR_STALL':'모터 회전 피드백 없음','CMD_TIMEOUT':'통신 워치독 정지','WIFI_LOSS':'무선 연결 끊김','BAD_PACKET':'잘못된 패킷',
 'SPEED_LIMIT':'속도 제한 위반','ENCODER_JUMP':'엔코더 급변','ENCODER_INVALID':'엔코더 값 오류','ODOMETRY_INVALID':'이동 추정 오류',
 'PATH_DEVIATION':'이동 경로 편차','CONTROL_TIMING':'제어 주기 오류','ESP32_REBOOT':'ESP32 재부팅','UDP_GUARD_FAILED':'UDP 전송 감시 오류',
 'ESP32_FAULT_OR_COMMAND_STALE':'ESP32 오류 또는 명령 유효 시간 초과','COMMAND_AGE_EXCEEDED':'명령 갱신 지연',
 'CAMERA_FAILED_OR_STALE':'카메라 오류 또는 영상 지연','TARGET_MISSING_OR_DUPLICATE_ID':'목표 누락 또는 중복 추적 ID','TARGET_MISSING':'목표 미검출',
 'TARGET_INVALID':'목표 관측 오류','PEOPLE_OVERLAP_ID_UNCERTAIN':'인물 겹침·목표 식별 불확실','TARGET_JUMP_UNCERTAIN':'목표 위치 또는 크기 급변',
 'RELATIVE_SIZE_UNRELIABLE':'상대 크기 추정 불안정','RADAR_UNKNOWN_OR_STALE':'레이더 미확인 또는 지연','RADAR_OBSTACLE':'후방 장애물 관측',
 'NOT_ARMED_OR_CONFIRMED':'제어 준비 또는 목표 확정 필요','ARMED_RESUME_REQUIRED':'제어 준비됨·명시적 재개 필요','CONFIRM_TARGET':'목표 확정 필요',
 'SELECT_TARGET':'목표를 선택하세요','EXPLICIT_RESUME_REQUIRED':'명시적 재개 필요','FOLLOW_ENABLED':'추종 허용됨','GESTURE_STOP':'손동작에 의한 정지',
 'PERCEPTION_STALE_OR_FAILED':'영상 인식 지연 또는 실패','COMMUNICATION_OR_ESP32_FAULT':'통신 또는 ESP32 오류',
 'LIVE_UDP':'실제 UDP 수신','MOCK_UDP':'모의 UDP 수신','LIVE_CAMERA':'실제 카메라','LOADING_MODELS':'모델 준비 중','LOADING_YOLO':'사람 검출 모델 준비 중',
 'LOADING_MEDIAPIPE':'손동작 모델 준비 중','WAITING_FOR_CAMERA':'카메라 대기','INFERENCING_YOLO':'사람 검출 처리 중','INFERENCING_GESTURE':'손동작 처리 중',
 'CAPTURING':'영상 수집 중','OPENING_CAMERA':'카메라 여는 중','CAMERA_ERROR':'카메라 오류','MODE_CHANGED_REARM_REQUIRED':'모드 변경·재활성화 필요'
}
FRAGMENTS={
 'UI ERROR / STOP LATCHED: ':'화면 오류로 정지 유지: ','LOGGING_FAILED: ':'로그 기록 실패: ',
 'CAMERA: ':'카메라 오류: ','VISION: ':'영상 인식 오류: ',
 'Relative height target: ':'상대 크기 목표: ',' (uncalibrated)':' (실측 미교정)','People ':'검출 인원 ',
 ' | Processing ':' | 처리 지연 ',' ms | Frame age ':' ms | 영상 경과 ','Safe rotation: ':'안전 회전 속도: ',
 '   target blue / measured green':'   파랑: 목표 / 초록: 측정','Session ':'세션 ',' / Trial ':' / 시험 ',
 ' visual result':' 육안 관찰',' rad/s | duration':' rad/s | 시험 시간','stop seq ':'정지 seq ', ' | TX error ':' | 송신 오류 ',
 'MANUAL SLOW':'수동 저속','SIMULATION SAFE':'시뮬레이션 안전 명령',' / MOTOR TEST ONLY':' / 수동 모터 시험 전용',
 'MANUAL ENABLED':'수동 시험 활성화','STALE / STOP':'영상 지연·정지','FRESH':'최신 영상','VISION DISCONNECTED FROM MOTORS':'영상 출력은 모터와 분리됨',
 'MULTIPLE PERSONS DETECTED - GESTURE CONTROL BLOCKED':'여러 사람 검출 — 손동작 제어 차단',
 'TELEMETRY SESSION / SEQ / STATE / CMD MATCH':'텔레메트리 세션·순서·상태·속도 일치','NOT OBSERVED':'미관측',
 'NO MOTOR TELEMETRY / NOT TESTED':'모터 텔레메트리 없음·미검증','FRESH STOPPED TELEMETRY VERIFIED':'최신 정지 응답 확인',
 'NOT VERIFIED':'미검증','VERIFIED BY FRESH TELEMETRY':'최신 텔레메트리로 확인',
 'CONNECTED / TELEMETRY VALID':'연결됨·유효 수신','DISCONNECTED / NO VALID TELEMETRY':'연결 끊김·유효 수신 없음',
 'UNKNOWN / NOT CONNECTED':'미확인·미연결','ACTION APPLIED':'동작 적용됨','True':'예','False':'아니오'
}


def ko(value):
    if not isinstance(value,str):return value
    if value in TEXT:return TEXT[value]
    if value.startswith('MOTOR_TEST_') and value[11:] in MEANINGS:return '모터 시험 · '+MEANINGS[value[11:]]+f' ({value})'
    if value.replace(' ','_') in MEANINGS and value not in MEANINGS:
        code=value.replace(' ','_');return MEANINGS[code]+f' ({code})'
    if value in MEANINGS:
        return MEANINGS[value]+(f' ({value})' if value not in ('NONE','UNKNOWN','PASS','FAIL','OK','NOT TESTED','NOT_TESTED') else '')
    for key in sorted(TEXT,key=len,reverse=True):
        if len(key)>18:value=value.replace(key,TEXT[key])
    for key in sorted(FRAGMENTS,key=len,reverse=True):value=value.replace(key,FRAGMENTS[key])
    pattern=r'(?<![A-Za-z0-9_(])('+ '|'.join(re.escape(k) for k in sorted(MEANINGS,key=len,reverse=True))+r')(?![A-Za-z0-9_])'
    return re.sub(pattern,lambda m:MEANINGS[m[0]]+f' ({m[0]})',value)


def add_choices(combo,choices):
    for choice in choices:combo.addItem(ko(choice),choice)
