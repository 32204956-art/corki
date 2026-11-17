from python_controller.models import MotionCommand
from python_controller.control.distance_estimator import relative_height_ratio


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class FollowController:
    def __init__(self, config, distance_estimator=relative_height_ratio):
        self.c = config
        self.distance_estimator = distance_estimator
        self.desired = config['desired_height_ratio']
        self.reset()

    def reset(self):
        self.error = None
        self.last = MotionCommand()
        self.last_time = None
        self.requested = MotionCommand()

    def request(self, target, width, height, error=None):
        c = self.c
        x1,y1,x2,y2 = target.bbox
        error = ((x1+x2)/2-width/2)/(width/2) if error is None else error
        size = self.distance_estimator(target,height)
        w = 0 if abs(error)<=c['horizontal_deadband'] else c['turn_sign']*c['turn_gain']*error
        size_error = self.desired-size
        vx = 0 if abs(size_error)<=c['height_deadband'] else c['distance_gain']*size_error
        vx *= max(0,1-abs(error)*2)
        return MotionCommand(vx*c['forward_sign'],0,w),error,size

    def compute(self, target, width, height, now, hardware=False):
        c = self.c
        x1, y1, x2, y2 = target.bbox
        raw_error = ((x1 + x2) / 2 - width / 2) / (width / 2)
        self.error = raw_error if self.error is None else c['filter_alpha'] * raw_error + (1-c['filter_alpha']) * self.error
        self.requested,_,size = self.request(target,width,height,self.error)
        vx,w = self.requested.vx_cm_s,self.requested.w_rad_s
        if hardware and vx*c['forward_sign'] < 0:
            vx = 0  # Stage 1 has no validated rear sensor integration.
        vx = clamp(vx, -c['max_linear_cm_s'], c['max_linear_cm_s'])
        w = clamp(w, -c['max_angular_rad_s'], c['max_angular_rad_s'])
        dt = .05 if self.last_time is None else clamp(now-self.last_time, 0, .1)
        def slew(current, desired, rate):
            if current * desired < 0:
                desired = 0
            return current + clamp(desired-current, -rate*dt, rate*dt)
        self.last = MotionCommand(slew(self.last.vx_cm_s,vx,c['linear_accel_cm_s2']),0,
                                  slew(self.last.w_rad_s,w,c['angular_accel_rad_s2']))
        self.last_time = now
        # A too-close target stops translation immediately rather than coasting.
        if size >= self.desired + c['height_deadband'] and self.last.vx_cm_s*c['forward_sign'] > 0:
            self.last = MotionCommand(0,0,self.last.w_rad_s)
        return self.last, self.error, size
