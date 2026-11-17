from pathlib import Path
from python_controller.models import PersonObservation


class PersonDetector:
    def __init__(self, config):
        from ultralytics import YOLO
        path = Path(config['model_path'])
        if not path.is_file():
            raise FileNotFoundError(f'Missing YOLO weights: {path}. Run python scripts/download_models.py')
        self.model, self.c = YOLO(str(path)), config

    def detect(self, image, timestamp):
        results = self.model.track(image,persist=True,tracker=self.c['tracker'],classes=[0],
                                  conf=self.c['confidence'],imgsz=self.c['image_size'],
                                  device=self.c['device'],verbose=False)
        boxes = results[0].boxes
        if boxes is None or boxes.id is None:
            return ()
        height,width = image.shape[:2]
        people = []
        for bbox, confidence, track_id in zip(boxes.xyxy.cpu().tolist(),boxes.conf.cpu().tolist(),boxes.id.int().cpu().tolist()):
            x1,y1,x2,y2 = bbox
            bbox = (max(0,x1),max(0,y1),min(width,x2),min(height,y2))
            if bbox[2]>bbox[0] and bbox[3]>bbox[1]:
                people.append(PersonObservation(track_id,bbox,confidence,timestamp))
        return tuple(people)
