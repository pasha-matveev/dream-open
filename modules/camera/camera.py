from modules.camera.camera_cv2 import CameraCV2
from picamera2 import Picamera2
import cv2 as cv


class Camera(CameraCV2):
    def __init__(self, args):
        super().__init__(args)
    
    def start(self):
        self.video = Picamera2()
        config = self.video.create_preview_configuration(main={"size": self.res})
        self.video.configure(config)
        self.video.start()
    
    def get_frame(self):
        yuv420 = self.video.capture_array("main")
        self.frame = cv.cvtColor(yuv420, cv.COLOR_YUV420p2RGB)