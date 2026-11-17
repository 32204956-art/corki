import math
from python_controller.models import ControlDecision, MotionCommand
from python_controller.control.follow_controller import FollowController, clamp
from python_controller.control.distance_estimator import relative_height_ratio


STATES = ('SEARCHING','TARGET_SELECTED','TRACKING','FOLLOWING','GESTURE_OVERRIDE',
          'PAUSED','TARGET_LOST','OBSTACLE_STOP','FAULT')


def iou(a, b):
    overlap = max(0,min(a[2],b[2])-max(a[0],b[0])) * max(0,min(a[3],b[3])-max(a[1],b[1]))
    area_a, area_b = (a[2]-a[0])*(a[3]-a[1]), (b[2]-b[0])*(b[3]-b[1])
    return overlap/max(1,area_a+area_b-overlap)


class GestureDebouncer:
    def __init__(self, config):
        self.c = config
        self.name = self.emitted = None
        self.since = self.last_time = self.neutral_since = None
        self.count = 0
        self.last_event = None
        self.approved_at = None

    def update(self, gesture, target_id, now):
        valid = (gesture is not None and gesture.target_id == target_id and
                 math.isfinite(gesture.confidence) and gesture.confidence >= self.c['confidence'] and
                 math.isfinite(gesture.timestamp) and 0 <= now-gesture.timestamp <= self.c['max_gap_s'] and
                 gesture.name in self.c['mapping'])
        if not valid:
            self.name, self.count = None, 0
            if self.neutral_since is None:
                self.neutral_since = now
            if now-self.neutral_since >= self.c['release_s']:
                self.emitted = None
            return None
        self.neutral_since = None
        if self.last_time is not None and gesture.timestamp <= self.last_time:
            return None
        if self.name != gesture.name or self.last_time is None or now-self.last_time > self.c['max_gap_s']:
            self.name, self.since, self.count = gesture.name, now, 0
        self.count += 1
        self.last_time = gesture.timestamp
        if self.count >= self.c['min_frames'] and now-self.since >= self.c['hold_s'] and self.emitted != self.name:
            self.emitted = self.name
            self.last_event = self.c['mapping'][self.name]
            self.approved_at = now
            return self.c['mapping'][self.name]
        return None


class Controller:
    def __init__(self, config, hardware=False):
        self.cfg, self.hardware = config, hardware
        self.follow = FollowController(config['follow'])
        self.debouncer = GestureDebouncer(config['gesture'])
        self.state, self.reason = 'SEARCHING', 'NOT_ARMED'
        self.target_id = None
        self.confirmed = self.armed = False
        self.previous = None
        self.resume_gesture_blocked = False
        self.resume_neutral_since = None
        self.estop_latched = False
        self.requested_command = None
        self.gesture_status = {}
        self.last_applied_action = self.last_applied_at = None

    def select_target(self, track_id):
        self.target_id, self.confirmed = track_id, False
        self.previous = None
        self.state, self.reason = 'TARGET_SELECTED', 'CONFIRM_TARGET'
        self.follow.reset()
        self.debouncer = GestureDebouncer(self.cfg['gesture'])

    def confirm_target(self):
        if self.target_id is not None:
            self.confirmed = True
            self.state, self.reason = 'TRACKING', 'EXPLICIT_RESUME_REQUIRED'

    def halt(self, state, reason, disarm=False):
        self.state, self.reason = state, reason
        if state in ('FAULT','TARGET_LOST','OBSTACLE_STOP') or reason == 'EMERGENCY_STOP':
            self.resume_gesture_blocked = True
            self.resume_neutral_since = None
        if disarm:
            self.armed = False
        self.follow.reset()
        return ControlDecision(self.state, MotionCommand(), reason, self.target_id)

    def step(self, frame, now, action=None, communication_ok=True, esp32_fault='NONE', radar=None):
        self.applied_action = None
        self.debouncer.last_event = None
        self.requested_command = None
        target = next((p for p in frame.people if p.track_id == self.target_id),None)
        if target is not None and frame.width>0 and frame.height>0 and all(math.isfinite(v) for v in target.bbox):
            self.requested_command = self.follow.request(target,frame.width,frame.height)[0]
        result = self._step(frame,now,action,communication_ok,esp32_fault,radar)
        if result.state in ('FOLLOWING','GESTURE_OVERRIDE'):
            self.requested_command = self.follow.requested
        g,b = frame.gesture,self.debouncer
        elapsed = max(0,now-b.since) if g is not None and b.name==g.name and b.since is not None else 0
        approved = bool(g is not None and b.name==g.name==b.emitted and b.count>=b.c['min_frames'] and elapsed>=b.c['hold_s'])
        rejection = None
        if g is None: rejection = 'NO_ELIGIBLE_GESTURE'
        elif g.target_id != self.target_id: rejection = 'HAND_TARGET_MISMATCH'
        elif g.confidence < b.c['confidence']: rejection = 'GESTURE_LOW_CONFIDENCE'
        elif g.name not in b.c['mapping']: rejection = 'GESTURE_NOT_MAPPED'
        elif not 0<=now-g.timestamp<=b.c['max_gap_s']: rejection = 'GESTURE_STALE'
        elif self.target_id is None: rejection = 'TARGET_NOT_SELECTED'
        elif not approved: rejection = 'GESTURE_UNSTABLE'
        elif not self.confirmed: rejection = 'TARGET_NOT_CONFIRMED'
        elif not self.armed and b.c['mapping'].get(g.name)!='PAUSE': rejection = 'CONTROLLER_NOT_ARMED'
        elif self.state in ('FAULT','TARGET_LOST','OBSTACLE_STOP') or self.estop_latched: rejection = 'SAFETY_STOP_ACTIVE'
        elif self.resume_gesture_blocked and b.c['mapping'].get(g.name)=='RESUME': rejection = 'RELEASE_RESUME_GESTURE'
        elif self.applied_action is None: rejection = 'ALREADY_APPLIED_OR_ACTION_NOT_ALLOWED'
        self.gesture_status = dict(confirmed=g.name if approved else None,confirmed_at=b.approved_at if approved else None,
                                   elapsed_s=elapsed,required_s=b.c['hold_s'],frames=b.count,required_frames=b.c['min_frames'],
                                   progress=min(1,elapsed/b.c['hold_s'],b.count/b.c['min_frames']),
                                   applied_action=self.applied_action,applied_at=now if self.applied_action else None,
                                   last_applied_action=self.last_applied_action,last_applied_at=self.last_applied_at,rejection=rejection)
        return result

    def apply_gesture(self, action, now):
        self.applied_action = self.last_applied_action = action
        self.last_applied_at = now

    def _step(self, frame, now, action=None, communication_ok=True, esp32_fault='NONE', radar=None):
        if action == 'ESTOP':
            self.estop_latched = True
        if self.estop_latched and action != 'ARM':
            return self.halt('PAUSED','EMERGENCY_STOP',True)
        if action == 'PAUSE':
            return self.halt('PAUSED','USER_PAUSE')
        s = self.cfg['safety']
        if not math.isfinite(now) or not frame.camera_ok or not math.isfinite(frame.timestamp) or not 0 <= now-frame.timestamp <= s['perception_timeout_s']:
            return self.halt('FAULT','CAMERA_FAILED_OR_STALE',True)
        if not communication_ok or esp32_fault != 'NONE':
            return self.halt('FAULT','COMMUNICATION_OR_ESP32_FAULT',True)
        if radar is not None and radar.obstacle_detected is True:
            return self.halt('OBSTACLE_STOP','RADAR_OBSTACLE')
        if radar is not None or s['require_radar']:
            if radar is None or not radar.data_valid or radar.obstacle_detected is None or not math.isfinite(radar.timestamp) or not 0 <= now-radar.timestamp <= s['perception_timeout_s']:
                return self.halt('OBSTACLE_STOP','RADAR_UNKNOWN_OR_STALE')
        if frame.width <= 0 or frame.height <= 0:
            return self.halt('FAULT','INVALID_IMAGE_SIZE',True)
        target = next((p for p in frame.people if p.track_id == self.target_id),None)
        if self.target_id is None:
            return self.halt('SEARCHING','SELECT_TARGET')
        if sum(p.track_id == self.target_id for p in frame.people) != 1:
            return self.halt('TARGET_LOST','TARGET_MISSING_OR_DUPLICATE_ID')
        if target is None:
            return self.halt('TARGET_LOST','TARGET_MISSING')
        box = target.bbox
        if (len(box) != 4 or not all(math.isfinite(v) for v in box) or
            not 0 <= box[0] < box[2] <= frame.width or not 0 <= box[1] < box[3] <= frame.height or
            not math.isfinite(target.confidence) or target.confidence < s['person_confidence'] or
            not math.isfinite(target.timestamp) or not 0 <= now-target.timestamp <= s['perception_timeout_s']):
            return self.halt('TARGET_LOST','TARGET_INVALID')
        f = self.cfg['follow']
        size = relative_height_ratio(target,frame.height)
        if not f['size_min'] <= size <= f['size_max']:
            return self.halt('TARGET_LOST','RELATIVE_SIZE_UNRELIABLE')
        if any(p.track_id != self.target_id and iou(box,p.bbox)>f['overlap_iou'] for p in frame.people):
            return self.halt('TARGET_LOST','PEOPLE_OVERLAP_ID_UNCERTAIN')
        if self.previous is not None:
            old = self.previous.bbox
            old_size = old[3]-old[1]
            if (abs((box[3]-box[1])/old_size-1)>f['size_jump_ratio'] or
                abs((box[0]+box[2]-old[0]-old[2])/2)/frame.width>f['center_jump_ratio']):
                self.previous = target
                return self.halt('TARGET_LOST','TARGET_JUMP_UNCERTAIN')
        self.previous = target
        event = self.debouncer.update(frame.gesture,self.target_id,now)
        if self.resume_gesture_blocked:
            g = frame.gesture
            resume_held = g is not None and g.target_id == self.target_id and self.cfg['gesture']['mapping'].get(g.name) == 'RESUME'
            if resume_held:
                self.resume_neutral_since = None
            else:
                if self.resume_neutral_since is None:
                    self.resume_neutral_since = now
                if now-self.resume_neutral_since >= self.cfg['gesture']['release_s']:
                    self.resume_gesture_blocked = False
            if event == 'RESUME':
                event = None
        stop_held = (frame.gesture is not None and frame.gesture.target_id == self.target_id and
                     frame.gesture.confidence >= self.cfg['gesture']['confidence'] and
                     self.debouncer.name == frame.gesture.name == self.debouncer.emitted and
                     self.cfg['gesture']['mapping'].get(frame.gesture.name) == 'PAUSE')
        if event == 'PAUSE' or stop_held:
            if event == 'PAUSE': self.apply_gesture('PAUSE',now)
            return self.halt('PAUSED','GESTURE_STOP')
        if action == 'ARM':
            self.estop_latched = False
            self.armed = True
            return self.halt('TRACKING' if self.confirmed else 'TARGET_SELECTED','ARMED_RESUME_REQUIRED')
        if not self.confirmed or not self.armed:
            return self.halt('TRACKING' if self.confirmed else 'TARGET_SELECTED','NOT_ARMED_OR_CONFIRMED')
        if action == 'RESUME' or event == 'RESUME':
            if event == 'RESUME': self.apply_gesture('RESUME',now)
            self.state, self.reason = 'FOLLOWING','FOLLOW_ENABLED'
        if event in ('CLOSER','FARTHER') and self.state in ('FOLLOWING','GESTURE_OVERRIDE'):
            self.apply_gesture(event,now)
            step = f['distance_step'] * (1 if event == 'CLOSER' else -1)
            self.follow.desired = clamp(self.follow.desired+step,f['size_min']+.1,f['size_max']-.1)
            self.state, self.reason = 'GESTURE_OVERRIDE', event
        if self.state not in ('FOLLOWING','GESTURE_OVERRIDE'):
            return self.halt(self.state,self.reason)
        command, error, size = self.follow.compute(target,frame.width,frame.height,now,self.hardware)
        return ControlDecision(self.state,command,self.reason,self.target_id,error,size)
