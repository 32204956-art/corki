import math
import time
from pathlib import Path
from python_controller.models import GestureObservation
from python_controller.control.state_machine import iou


def gesture_diagnostics(raw, target_id, people_count, timestamp, hand_count):
    reason = None
    if not hand_count: reason = 'NO_HAND_DETECTED'
    elif people_count > 1: reason = 'MULTIPLE_PERSON_RESTRICTION'
    elif target_id is None: reason = 'TARGET_NOT_SELECTED'
    elif not raw: reason = 'NO_GESTURE_CLASSIFICATION'
    elif not any(owner == target_id for _,_,owner in raw): reason = 'HAND_TARGET_MISMATCH'
    return dict(raw=[dict(name=name,confidence=score,associated_target=owner) for name,score,owner in raw],
                hand_count=hand_count,rejection=reason,recognized_at=timestamp)


def associate_hand(points, people):
    # shortcut: spatial ownership is conservative, not biometric identity; use pose wrists in Stage 2.
    if len(points) != 21 or not all(math.isfinite(x) and math.isfinite(y) for x,y in points):
        return None
    anchors = [points[i] for i in (0,5,9,13,17)]
    owners = [p for p in people if all(p.bbox[0]<=x<=p.bbox[2] and p.bbox[1]<=y<=p.bbox[3] for x,y in anchors)]
    if len(owners)!=1:
        return None
    owner = owners[0]
    if any(p.track_id != owner.track_id and iou(owner.bbox,p.bbox)>.1 for p in people):
        return None
    return owner.track_id


def choose_gesture(raw, target_id, timestamp, mapping=None, people_count=1):
    # Unvalidated cross-person hand ownership never grants control in Stage 1.
    if people_count != 1:
        return None
    mapping = mapping or {'Closed_Fist':'PAUSE'}
    owned = [(name,score) for name,score,owner in raw if owner is not None and owner == target_id and math.isfinite(score)]
    if not owned:
        return None
    stops = [(n,s) for n,s in owned if mapping.get(n)=='PAUSE']
    if stops:
        name,score = max(stops,key=lambda v:v[1])
    elif len({n for n,_ in owned}) == 1:
        name,score = max(owned,key=lambda v:v[1])
    else:
        return None
    return GestureObservation(name,score,target_id,timestamp)


class GestureRecognizer:
    def __init__(self, config, gesture_config):
        import mediapipe as mp
        from mediapipe.tasks.python import vision
        path = Path(config['gesture_model_path'])
        if not path.is_file():
            raise FileNotFoundError(f'Missing gesture model: {path}. Run python scripts/download_models.py')
        self.mp, self.mapping = mp, gesture_config['mapping']
        options = vision.GestureRecognizerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(path)),
            running_mode=vision.RunningMode.VIDEO,num_hands=config['max_hands'],
            min_hand_detection_confidence=.5,min_hand_presence_confidence=.5,min_tracking_confidence=.5)
        self.recognizer = vision.GestureRecognizer.create_from_options(options)
        self.last_ms = -1
        self.diagnostics = gesture_diagnostics([],None,0,0,0)

    def recognize(self, image, people, target_id, timestamp):
        import cv2
        started_at=time.monotonic()
        rgb = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        mp_image = self.mp.Image(image_format=self.mp.ImageFormat.SRGB,data=rgb)
        ms = max(self.last_ms+1,int(timestamp*1000))
        self.last_ms = ms
        result = self.recognizer.recognize_for_video(mp_image,ms)
        height,width = image.shape[:2]
        raw, hands = [], []
        for landmarks, categories in zip(result.hand_landmarks,result.gestures):
            points = [(p.x*width,p.y*height) for p in landmarks]
            owner = associate_hand(points,people)
            hands.append((points,owner))
            if categories:
                top = max(categories,key=lambda c:c.score)
                raw.append((top.category_name,top.score,owner))
        self.diagnostics = gesture_diagnostics(raw,target_id,len(people),timestamp,len(hands))
        self.diagnostics.update(frame_at=timestamp,recognition_started_at=started_at,recognized_at=time.monotonic())
        selected = choose_gesture(raw,target_id,timestamp,self.mapping,people_count=len(people))
        if selected is None and self.diagnostics['rejection'] is None:
            self.diagnostics['rejection'] = 'CONFLICTING_OR_UNSUPPORTED_GESTURES'
        return selected, hands

    def close(self):
        self.recognizer.close()
