from modules.camera.camera_cv2 import CameraCV2
from picamera2 import Picamera2
import cv2 as cv


class Camera(CameraCV2):
    def __init__(self, args):
        super().__init__(args)
    
    def start(self):
        self.video = Picamera2()
        print('Camera resultion:', self.res)
        config = self.video.create_preview_configuration(main={"size": self.res})
        print('Camera Config:', config["main"]["format"])
        self.video.configure(config)
        self.video.start()
        self.is_opened = True
    
    def get_frame(self):
        frame = self.video.capture_array("main")
        self.frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
#         self.frame = cv.cvtColor(yuv420, cv.COLOR_YUV420p2RGB)
