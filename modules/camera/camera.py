from modules.camera.camera_cv2 import CameraCV2
from picamera2 import Picamera2
import cv2 as cv
import queue


class Camera(CameraCV2):
    def __init__(self, args):
        super().__init__(args)
    
    def start(self):
        self.video = Picamera2()
        config = self.video.create_preview_configuration(main={"size": self.res})
        self.video.configure(config)
        self.video.post_callback = self._post_callback
        self.video.start()
    
    def _post_callback(self, req):
        frame = req.make_array("main")
        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        try:
            self.q.put_nowait(frame)
        except queue.Full:
            pass
    
    def get_frame(self):
        frame = self.video.capture_array("main")
        self.frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        # self.frame = cv.cvtColor(yuv420, cv.COLOR_YUV420p2RGB)

    def stop(self):
        self.stop_event.set()
        cv.destroyAllWindows()