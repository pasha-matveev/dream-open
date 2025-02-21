import cv2 as cv
import argparse
from picamera2 import Picamera2
from serial import Serial
import math

max_value = 255
max_value_H = 360//2
low_H = 104
low_S = 167
low_V = 102
high_H = 122
high_S = 237
high_V = 216
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

mirror_center = [320, 240]
ball_angle = 0

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


# serial = Serial(port='???', boudrate=115200, timeout=1)

# if serial.is_open:
#     print('USB is working')
# else:
#     print('Failed to open port')

while True:
    yuv420 = picam2.capture_array("lores")
    frame = cv.cvtColor(yuv420, cv.COLOR_YUV420p2RGB)
    frame_HSV = cv.cvtColor(frame, cv.COLOR_RGB2HSV)
    frame_threshold = cv.inRange(
        frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

    contours, hierarchy = cv.findContours(frame_threshold, 1, 2)

    cv.drawMarker(frame, mirror_center, (0, 0, 0), cv.MARKER_CROSS, 10, 2)

    if len(contours):
        cnt = max(contours, key=lambda x: cv.contourArea(x))
        M = cv.moments(cnt)

        (x, y), ball_radius = cv.minEnclosingCircle(cnt)
        ball_center = (int(x), int(y))
        ball_radius = int(ball_radius)
        cv.circle(frame, ball_center, ball_radius, (0, 0, 255), 2)
        cv.drawMarker(frame, ball_center, (0, 0, 255), cv.MARKER_CROSS, 10, 2)
        cv.line(frame, ball_center, mirror_center, (0, 0, 0), 2)

    ball_angle = math.degrees(math.atan2(ball_center[1] - mirror_center[1], ball_center[0] - mirror_center[0]))

    print(ball_angle)

    # serial.write(str(ball_angle).encode())

    cv.imshow(window_capture_name, frame)
    cv.imshow(window_detection_name, frame_threshold)

    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break
