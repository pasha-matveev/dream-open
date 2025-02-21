# import cv2 as cv
# import numpy as np

# class CVCamera:
#     def __init__(self, settings):
#         self.settings = settings
#         self.center = self.settings['camera']['center']
#         self.radius = self.settings['camera']['radius']
#         self.ball_threshold = self.settings['camera']['objects'][0]['threshold']
#         self.yellow_goal_threshold = self.settings['camera']['objects'][1]['threshold']
#         self.blue_goal_threshold = self.settings['camera']['objects'][2]['threshold']
#         self.frame = None
#         self.start()
    
#     def start(self):
#         self.video = cv.VideoCapture(0)
    
#     def read(self):
#         ret, self.frame = self.video.read()
    
#     def preview(self):
#         pass
    
#     def stop(self):
#         pass

#!/usr/bin/python3

import cv2

from picamera2 import Picamera2

cv2.startWindowThread()

picam2 = Picamera2()
config = picam2.create_preview_configuration(lores={"size": (640, 480)})
picam2.configure(config)
picam2.start()

while True:
    yuv420 = picam2.capture_array("lores")
    rgb = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2RGB)
    cv2.imshow("Camera", rgb)