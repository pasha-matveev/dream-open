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

# import cv2

# from picamera2 import Picamera2

# cv2.startWindowThread()

# picam2 = Picamera2()
# config = picam2.create_preview_configuration(lores={"size": (640, 480)})
# picam2.configure(config)
# picam2.start()

# while True:
#     yuv420 = picam2.capture_array("lores")
#     rgb = cv2.cvtColor(yuv420, cv2.COLOR_YUV420p2RGB)
#     cv2.imshow("Camera", rgb)


from __future__ import print_function
import cv2 as cv
import argparse
from picamera2 import Picamera2


max_value = 255
max_value_H = 360//2
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'


def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv.setTrackbarPos(low_H_name, window_detection_name, low_H)


def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv.setTrackbarPos(high_H_name, window_detection_name, high_H)


def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv.setTrackbarPos(low_S_name, window_detection_name, low_S)


def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv.setTrackbarPos(high_S_name, window_detection_name, high_S)


def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv.setTrackbarPos(low_V_name, window_detection_name, low_V)


def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv.setTrackbarPos(high_V_name, window_detection_name, high_V)


# parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
# parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
# args = parser.parse_args()
# cap = cv.VideoCapture(args.camera)

picam2 = Picamera2()
config = picam2.create_preview_configuration(lores={"size": (640, 480)})
picam2.configure(config)
picam2.start()


cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)
cv.createTrackbar(low_H_name, window_detection_name, low_H,
                  max_value_H, on_low_H_thresh_trackbar)
cv.createTrackbar(high_H_name, window_detection_name, high_H,
                  max_value_H, on_high_H_thresh_trackbar)
cv.createTrackbar(low_S_name, window_detection_name, low_S,
                  max_value, on_low_S_thresh_trackbar)
cv.createTrackbar(high_S_name, window_detection_name, high_S,
                  max_value, on_high_S_thresh_trackbar)
cv.createTrackbar(low_V_name, window_detection_name, low_V,
                  max_value, on_low_V_thresh_trackbar)
cv.createTrackbar(high_V_name, window_detection_name, high_V,
                  max_value, on_high_V_thresh_trackbar)


while True:

    # ret, frame = cap.read()
    # if frame is None:
    #     break

    yuv420 = picam2.capture_array("lores")
    frame = cv.cvtColor(yuv420, cv.COLOR_YUV420p2RGB)
    frame_HSV = cv.cvtColor(frame, cv.COLOR_RGB2HSV)
    frame_threshold = cv.inRange(
        frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

    contours, hierarchy = cv.findContours(frame_threshold, 1, 2)

    if len(contours):
        cnt = max(contours, key=lambda x: cv.contourArea(x))
        M = cv.moments(cnt)

        (x, y), radius = cv.minEnclosingCircle(cnt)
        center = (int(x), int(y))
        radius = int(radius)
        cv.circle(frame, center, radius, (255, 0, 0), 2)

    cv.imshow(window_capture_name, frame)
    cv.imshow(window_detection_name, frame_threshold)

    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break
