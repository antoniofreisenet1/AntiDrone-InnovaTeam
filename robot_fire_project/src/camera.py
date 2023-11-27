import cv2

class Camera:
    def __init__(self):
        # Initialiser la caméra
        self.cap = cv2.VideoCapture(0)

    def get_frame(self):
        # Capturer un cadre de la caméra
        ret, frame = self.cap.read()
        return frame if ret else None
