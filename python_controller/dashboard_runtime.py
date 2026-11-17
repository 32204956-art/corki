"""Latest-only capture/inference workers and UI-independent dashboard controller."""
from dataclasses import asdict
import copy
import math
from pathlib import Path
import threading
import time

from python_controller.control.state_machine import Controller
from python_controller.models import PerceptionFrame,PersonObservation,GestureObservation
from python_controller.monitoring import SessionLogger,TestBook,motor_feedback
from python_controller.vision.person_tracker import TargetSelector
from python_controller.vision.gesture_recognizer import gesture_diagnostics


class LatestSlot:
    def __init__(self):
        self.lock=threading.Lock()
        self.value=None
        self.version=0

    def put(self,value):
        with self.lock:
            self.value=value;self.version+=1

    def get(self):
        with self.lock: return self.version,self.value


class VisionStream:
    def __init__(self,config):
        self.cfg=config
        self.capture,self.results=LatestSlot(),LatestSlot()
        self.done=threading.Event()
        self.error=None
        self.status='LOADING_MODELS'
        self.camera_status='OPENING_CAMERA'
        self.threads=[]

    def start(self):
        for method,name in [(self._camera,'camera-latest'),(self._vision,'vision-latest')]:
            thread=threading.Thread(target=method,name=name,daemon=True)
            self.threads.append(thread);thread.start()

    def _camera(self):
        camera=None
        try:
            from python_controller.vision.camera import Camera
            camera=Camera(self.cfg['camera'])
            self.camera_status='CAPTURING'
            while not self.done.is_set():
                image,timestamp=camera.read()
                self.capture.put((image,timestamp))
        except Exception as exc:
            self.camera_status='CAMERA_ERROR'
            self.error=f'CAMERA: {type(exc).__name__}: {exc}'
        finally:
            if camera: camera.close()

    def _vision(self):
        recognizer=None
        try:
            from python_controller.vision.person_detector import PersonDetector
            from python_controller.vision.gesture_recognizer import GestureRecognizer
            self.status='LOADING_YOLO'
            detector=PersonDetector(self.cfg['vision'])
            self.status='LOADING_MEDIAPIPE'
            recognizer=GestureRecognizer(self.cfg['vision'],self.cfg['gesture'])
            version=0;last_completed=None
            self.status='WAITING_FOR_CAMERA'
            while not self.done.is_set():
                current,value=self.capture.get()
                if value is None or current==version:
                    self.done.wait(.005);continue
                version=current
                image,timestamp=value
                self.status='INFERENCING_YOLO'
                people=detector.detect(image,timestamp)
                target_id=getattr(self,'target_id',None)
                self.status='INFERENCING_GESTURE'
                gesture,hands=recognizer.recognize(image,people,target_id,timestamp)
                completed=time.monotonic()
                self.results.put(dict(frame=PerceptionFrame(timestamp,image.shape[1],image.shape[0],people,gesture),
                                      image=image,hands=hands,vision=recognizer.diagnostics,
                                      completed_at=completed,latency_ms=(completed-timestamp)*1000,
                                      fps=1/(completed-last_completed) if last_completed else None))
                last_completed=completed
                self.status='RUNNING'
        except Exception as exc:
            self.error=f'VISION: {type(exc).__name__}: {exc}'
        finally:
            if recognizer: recognizer.close()

    def latest(self):
        return self.results.get()[1]

    def close(self):
        self.done.set()
        for thread in self.threads: thread.join(timeout=.5)


class PreviewStream:
    """Explicit synthetic UI preview; never eligible for hardware connection."""
    error=None
    status='SYNTHETIC_PREVIEW'
    target_id=None
    def start(self): pass
    def latest(self):
        import numpy as np
        now=time.monotonic()
        people=(PersonObservation(7,(220,65,420,410),.94,now),)
        g=GestureObservation('Open_Palm',.92,7,now) if self.target_id==7 else None
        return dict(frame=PerceptionFrame(now,640,480,people,g),image=np.full((480,640,3),(31,39,48),dtype=np.uint8),
                    hands=[],vision=gesture_diagnostics([('Open_Palm',.92,7)],self.target_id,1,now,1),
                    completed_at=now,latency_ms=None,fps=None)
    def close(self): pass


class DashboardEngine:
    def __init__(self,config,folder,source='LIVE_CAMERA',client_factory=None,logger=True):
        self.cfg=config
        self.source=source
        self.c=Controller(config)
        self.client=None
        self.client_factory=client_factory
        self.pending=None
        self.last_result=None
        self.snapshot=None
        self.notice='Select a target, confirm, arm, then resume.'
        self.book=TestBook()
        self.logger=SessionLogger(folder,'SIMULATION',source) if logger else None
        if self.logger:self.logger.save_tests(self.book)
        self.error=None
        self.last_motor_snapshot=None
        self.connection_attempt=None
        self.last_transport=None

    @property
    def mode(self): return 'HARDWARE' if self.client else 'SIMULATION'

    def connect_hardware(self,host,acknowledged):
        # Compatibility entry point; automatic hardware following is no longer exposed.
        if self.cfg['communication']['signs_verified'] is not True:
            raise ValueError('Physical signs are unverified; use the MOTOR TEST connection instead')
        self.connect_motor_test(host,acknowledged)

    def connect_motor_test(self,host,acknowledged):
        if self.source!='LIVE_CAMERA': raise ValueError('Synthetic observations cannot enable HARDWARE')
        if acknowledged is not True: raise ValueError('Explicit hardware authorization required')
        if self.client: raise ValueError('Already connected; stop and disconnect first')
        from python_controller.communication.motor_test import MotorTestClient
        config=copy.deepcopy(self.cfg);config['communication']['host']=host
        self.connection_attempt=dict(status='PENDING',host=host,phase='CONFIG',checked_at=time.time())
        try:
            client=(self.client_factory or MotorTestClient)(config)
            client.start()  # STOP proof followed by stationary heartbeat. No motor authorization.
        except Exception as exc:
            self.connection_attempt.update(status='FAILED',phase=getattr(locals().get('client'),'startup_phase','CONFIG'),
                                           error_code=getattr(exc,'winerror',None) or getattr(exc,'errno',None),error_type=type(exc).__name__)
            if self.logger:
                failed_client=locals().get('client')
                if hasattr(failed_client,'drain_transport_events'):
                    for event in failed_client.drain_transport_events():self.logger.write(event,True)
                self.logger.write(dict(event='CONNECTION_FAILED',attempt=dict(self.connection_attempt)),True)
            raise
        self.connection_attempt.update(status='STARTED',phase='RX_BOUND')
        self.client=client
        self.last_motor_snapshot=None
        self.notice='MOTOR TEST ONLY: lift wheels and authorize one burst; A/R and gesture hardware output are disabled'
        self.c.hardware=True
        self.c.halt('PAUSED','MODE_CHANGED_REARM_REQUIRED',True)
        if self.logger:
            self.logger.mode='HARDWARE'
            self.logger.write(dict(event='MODE_CHANGED',state='PAUSED',control_source='MANUAL_MOTOR_TEST'),True)

    def _manual_client(self):
        if not self.client or not getattr(self.client,'manual_test_only',False):
            raise ValueError('Connect the MOTOR TEST UDP controller first')
        return self.client

    def authorize_motor_test(self,lifted,approved):
        client=self._manual_client()
        client.authorize_manual(lifted,approved)
        self.c.estop_latched=False
        self.c.halt('PAUSED','MANUAL_MOTOR_TEST_ONLY',True)

    def begin_motor_test(self,direction,duration):
        self._manual_client().begin_test(direction,duration)

    def _drain_motor_events(self):
        if self.client and hasattr(self.client,'drain_transport_events'):
            for event in self.client.drain_transport_events():
                if self.logger:self.logger.write(event,True)
        if self.client and getattr(self.client,'manual_test_only',False):
            for event in self.client.drain_events():
                if self.logger:self.logger.write(event,True)

    def disconnect(self):
        if self.client:
            self.last_transport=self.client.snapshot() if hasattr(self.client,'snapshot') else None
            self.client.close()
            self._drain_motor_events()
            if getattr(self.client,'manual_test_only',False):
                self.last_motor_snapshot=self.client.snapshot().get('motor_test')
        self.client=None
        self.c.hardware=False
        self.c.halt('PAUSED','MODE_CHANGED_REARM_REQUIRED',True)
        if self.logger: self.logger.mode='SIMULATION'
        self.notice='SIMULATION: select a target, confirm, arm, then resume'

    def request(self,action,target_id=None):
        if action=='ESTOP':
            if self.client: self.client.stop('EMERGENCY_STOP')
            self.c.estop_latched=True
            self.c.halt('PAUSED','EMERGENCY_STOP',True)
            self.pending=('ESTOP',None)
        elif action=='PAUSE' and self.client and getattr(self.client,'manual_test_only',False):
            self.client.stop('USER_STOP');self.pending=('PAUSE',None)
        elif self.pending is None or self.pending[0]!='ESTOP':
            self.pending=(action,target_id)

    def tick(self,result,now=None,pipeline_error=None,pipeline_health=None):
        now=time.monotonic() if now is None else now
        previous_state=self.snapshot['state'] if self.snapshot else None
        if result is not None: self.last_result=result
        r=self.last_result
        f=r['frame'] if r else PerceptionFrame(now,640,480,(),camera_ok=False)
        if pipeline_error:
            f=PerceptionFrame(f.timestamp,f.width,f.height,f.people,f.gesture,False)
        action,target=self.pending or (None,None)
        self.pending=None
        if action=='SELECT': self.c.select_target(target);action=None
        elif action=='CLEAR': self.c.select_target(None);action='PAUSE'
        elif action=='CONFIRM': self.c.confirm_target();action=None
        if self.c.target_id is None:
            candidate=TargetSelector.candidate(f.people)
            if candidate is not None: self.c.select_target(candidate)
        transport=self.client.snapshot() if self.client else dict(connected=False,ready=False,armed=False,fault=None,reason='SIMULATION_NO_UDP',
            telemetry=None,telemetry_valid=False,last_rx_at=None,telemetry_age_s=None,last_tx=None,last_tx_at=None,
            tx_error=None,acknowledged_command=None,stop_verified=False,stop_requested_at=None,stop_seq=None)
        manual=bool(self.client and getattr(self.client,'manual_test_only',False))
        if manual:
            if not self.error:self.client.operator_alive()
            if action in ('ARM','RESUME'):
                self.notice='AUTOMATIC_HARDWARE_FOLLOWING_DISABLED: use separate MOTOR TEST authorization'
                action=None
        if action=='ARM' and self.client and not self.client.arm():
            action=None
            self.notice='ARM_DENIED: fresh, matching STOPPED telemetry required'
        if self.client: transport=self.client.snapshot()
        comm_ok=not self.client or (transport['connected'] and (not self.c.armed or transport['armed']))
        decision=self.c.step(f,now,action if not manual or action=='ESTOP' else 'PAUSE',comm_ok,transport['fault'] if self.client else 'NONE')
        if self.client: self.client.submit(decision,f.timestamp)
        target=next((p for p in f.people if p.track_id==self.c.target_id),None)
        fresh=bool(f.camera_ok and 0<=now-f.timestamp<=self.cfg['safety']['perception_timeout_s'])
        vision=copy.deepcopy(r['vision']) if r else gesture_diagnostics([],self.c.target_id,0,now,0)
        gesture=dict(self.c.gesture_status)
        if vision.get('rejection'): gesture['rejection']=vision['rejection']
        if not fresh: gesture.update(confirmed=None,progress=0,rejection='PERCEPTION_STALE_OR_FAILED')
        if self.c.state in ('FAULT','TARGET_LOST','OBSTACLE_STOP') or self.c.estop_latched:
            gesture['rejection']='SAFETY_STOP_ACTIVE' if not vision.get('rejection') else vision['rejection']
        telemetry=transport['telemetry']
        requested=asdict(self.c.requested_command) if self.c.requested_command else None
        safe=asdict(decision.command)
        motor=transport.get('motor_test') or self.last_motor_snapshot
        if manual:
            gesture.update(applied_action=None,rejection='MANUAL_MOTOR_TEST_ONLY')
            safe=transport['motor_test']['command']
            requested=safe.copy()
        center=((target.bbox[0]+target.bbox[2])/2,(target.bbox[1]+target.bbox[3])/2) if target else None
        data=dict(state=decision.state,stop_reason=decision.reason,target_id=self.c.target_id,
            armed=self.c.armed,following=decision.state in ('FOLLOWING','GESTURE_OVERRIDE'),
            target_confirmed=self.c.confirmed,target_lost=decision.state=='TARGET_LOST',
            detected_people=len(f.people),detection_confidence=target.confidence if target else None,
            target_last_seen=self.c.previous.timestamp if self.c.previous else None,target_center=center,
            horizontal_error=(center[0]-f.width/2)/(f.width/2) if center else None,
            relative_size=(target.bbox[3]-target.bbox[1])/f.height if target else None,
            desired_height_ratio=self.c.follow.desired,requested=requested,safe=safe,
            limits_or_stop_applied=requested is not None and requested!=safe,vision=vision,gesture=gesture,
            raw_gesture=vision['raw'],confirmed_gesture=gesture.get('confirmed'),applied_action=gesture.get('applied_action'),
            gesture_rejection_reason=gesture.get('rejection'),transport=transport,
            telemetry=telemetry,motors=motor_feedback(telemetry,self.cfg['communication']['wheel_radius_cm']),
            fps=r['fps'] if r else None,processing_ms=r['latency_ms'] if r else None,
            frame_age_s=max(0,now-f.timestamp) if r else None,perception_fresh=fresh,
            frame_at=f.timestamp if r else None,vision_completed_at=r['completed_at'] if r else None,
            program_mode=self.mode,evidence_source=self.source,notice=pipeline_error or self.notice,pipeline_health=pipeline_health,
            control_source='MANUAL_MOTOR_TEST' if manual else 'VISION_SIMULATION',motor_test=motor)
        if manual:
            data.update(state='MOTOR_TEST_'+motor['status'],stop_reason=transport['reason'],armed=transport['manual_enabled'],following=False)
        self.snapshot=data
        if self.client:self.last_transport=transport
        if self.logger:
            try:
                self._drain_motor_events()
                self.logger.write(data,force=action in ('ESTOP','ARM','RESUME','PAUSE') or bool(gesture.get('applied_action')) or data['state']!=previous_state)
            except Exception as exc:
                self.error=f'LOGGING_FAILED: {type(exc).__name__}'
                self.request('ESTOP')
        return data

    def close(self):
        if self.client:
            self.client.close();self._drain_motor_events();self.client=None
        if self.logger:
            self.logger.save_tests(self.book)
            self.logger.close()
