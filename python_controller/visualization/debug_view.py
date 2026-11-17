class DebugView:
    NAME = 'Omni Follow - SIMULATION first'

    def __init__(self):
        import cv2
        self.cv2 = cv2
        self.events = []
        self.image_width = 640
        cv2.namedWindow(self.NAME,cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.NAME,self.on_mouse)

    def on_mouse(self, event, x, y, flags, param):
        if event == self.cv2.EVENT_LBUTTONDOWN:
            if x >= self.image_width and y < 55:
                self.events.append(('ESTOP',))
            elif x < self.image_width:
                self.events.append(('SELECT',x,y))

    def drain_events(self):
        events,self.events = self.events,[]
        return events

    def draw(self,image,people,hands,decision,gesture,armed,desired,snapshot,fps,latency,candidate,mode):
        import numpy as np
        cv = self.cv2
        height,width = image.shape[:2]
        self.image_width = width
        canvas = np.zeros((max(height,640),width+500,3),dtype=np.uint8)
        canvas[:height,:width] = image
        for p in people:
            target = p.track_id == decision.target_id
            color = (0,220,255) if target else (60,220,60)
            x1,y1,x2,y2 = map(int,p.bbox)
            cv.rectangle(canvas,(x1,y1),(x2,y2),color,2)
            cv.putText(canvas,f'{"TARGET" if target else "PERSON"} {p.track_id} {p.confidence:.2f}',(x1,max(18,y1-6)),cv.FONT_HERSHEY_SIMPLEX,.5,color,1)
            if target: cv.circle(canvas,((x1+x2)//2,(y1+y2)//2),5,color,-1)
        cv.line(canvas,(width//2,0),(width//2,height),(120,120,120),1)
        for points,owner in hands:
            for x,y in points:
                cv.circle(canvas,(int(x),int(y)),2,(0,220,255) if owner==decision.target_id else (120,120,120),-1)
        cv.rectangle(canvas,(width,0),(width+499,54),(30,30,185),-1)
        cv.putText(canvas,'EMERGENCY STOP [SPACE / E]',(width+12,34),cv.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2)
        cmd = decision.command
        lines = [f'Mode: {mode}',f'State: {decision.state}',f'People: {len(people)}  Candidate: {candidate}',
                 f'Target ID: {decision.target_id}',f'Armed: {armed}  Following: {decision.state in ("FOLLOWING","GESTURE_OVERRIDE")}',
                 f'Gesture: {gesture.name if gesture else ("DISABLED: MULTIPLE PEOPLE" if len(people)>1 else "NONE / UNASSOCIATED")}',
                 f'Gesture confidence: {gesture.confidence:.2f}' if gesture else 'Gesture confidence: --',
                 f'Horizontal error: {decision.horizontal_error:+.3f}',
                 f'Relative height: {decision.relative_size:.3f}' if decision.relative_size is not None else 'Relative height: --',
                 f'Desired height ratio: {desired:.2f} (uncalibrated)',
                 f'vx: {cmd.vx_cm_s:+.2f} cm/s  vy: {cmd.vy_cm_s:+.2f}',f'w: {cmd.w_rad_s:+.3f} rad/s',
                 f'ESP32: {"CONNECTED" if snapshot["connected"] else "DISCONNECTED"}',
                 f'Transport: {snapshot["reason"]}', 'Rear radar: UNKNOWN / NOT CONNECTED',
                 f'FPS: {fps:.1f}  Processing: {latency:.1f} ms',f'Stop / status: {decision.reason}',
                 'Click person -> ENTER confirms target','A: arm  R / Open Palm: resume','P / Closed Fist: pause  C: clear target',
                 'Thumb Up/Down: change relative setpoint','Q / ESC: stop and exit']
        for i,line in enumerate(lines):
            cv.putText(canvas,line,(width+10,80+i*25),cv.FONT_HERSHEY_SIMPLEX,.48,(225,225,225),1,cv.LINE_AA)
        return canvas

    def show(self,canvas):
        self.cv2.imshow(self.NAME,canvas)

    def closed(self):
        return self.cv2.getWindowProperty(self.NAME,self.cv2.WND_PROP_VISIBLE)<1

    def close(self):
        self.cv2.destroyAllWindows()
