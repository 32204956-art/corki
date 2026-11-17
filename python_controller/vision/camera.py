import time


class Camera:
    """OpenCV is the only platform-dependent capture adapter in Stage 1."""
    def __init__(self, config):
        import cv2
        self.cv2, self.c = cv2, config
        backend = {'AUTO':cv2.CAP_ANY,'DSHOW':cv2.CAP_DSHOW,'MSMF':cv2.CAP_MSMF,'V4L2':cv2.CAP_V4L2}
        if config['backend'] not in backend:
            raise ValueError('Unknown camera backend')
        self.cap = cv2.VideoCapture(config['index'],backend[config['backend']])
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError('Camera open failed; check index, privacy permission and backend')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,config['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,config['height'])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,1)

    def read(self):
        # Timestamp BEFORE read: a blocked capture may not grant a fresh motion lease.
        timestamp = time.monotonic()
        ok, image = self.cap.read()
        if not ok or image is None:
            raise RuntimeError('Camera frame read failed')
        if self.c['mirror']:
            image = self.cv2.flip(image,1)
        return image,timestamp

    def close(self):
        self.cap.release()
