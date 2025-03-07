import cv2 as cv
import numpy as np
import math


class TrackingObject:
    def __init__(self, settings, name):
        self.settings = settings
        self.name = name
        self.threshold = settings['objects'][self.name]['threshold']
        self.mirror_center = (settings['outer_radius'], 
                              settings['outer_radius'])
        self.low_H = self.threshold[0]
        self.high_H = self.threshold[1]
        self.low_S = self.threshold[2]
        self.high_S = self.threshold[3]
        self.low_V = self.threshold[4]
        self.high_V = self.threshold[5]
        self.frame_threshold = None
        self.visible = False
        self.min_area = 0

        self.center = None
        self.angle = None
        self.dist = None

    def start_preview_detection(self):
        max_value = 255
        max_value_H = 360//2
        self.window_detection_name = f'{self.name} detection'
        cv.namedWindow(self.window_detection_name)
        cv.createTrackbar(f'Low H', self.window_detection_name, self.low_H,
                          max_value_H, self.on_low_H_thresh_trackbar)
        cv.createTrackbar(f'High H', self.window_detection_name, self.high_H,
                          max_value_H, self.on_high_H_thresh_trackbar)
        cv.createTrackbar(f'Low S', self.window_detection_name, self.low_S,
                          max_value, self.on_low_S_thresh_trackbar)
        cv.createTrackbar(f'High S', self.window_detection_name, self.high_S,
                          max_value, self.on_high_S_thresh_trackbar)
        cv.createTrackbar(f'Low V', self.window_detection_name, self.low_V,
                          max_value, self.on_low_V_thresh_trackbar)
        cv.createTrackbar(f'High V', self.window_detection_name, self.high_V,
                          max_value, self.on_high_V_thresh_trackbar)

    def preview_detection(self):
        if self.frame_threshold is not None:
            cv.imshow(self.window_detection_name, self.frame_threshold)

    def on_low_H_thresh_trackbar(self, val: int):
        self.low_H = val
        self.low_H = max(1, min(self.high_H-1, self.low_H))

    def on_high_H_thresh_trackbar(self, val):
        self.high_H = val
        self.high_H = max(self.high_H, self.low_H+1)

    def on_low_S_thresh_trackbar(self, val):
        self.low_S = val
        self.low_S = max(1, min(self.high_S-1, self.low_S))

    def on_high_S_thresh_trackbar(self, val):
        self.high_S = val
        self.high_S = max(self.high_S, self.low_S+1)

    def on_low_V_thresh_trackbar(self, val):
        self.low_V = val
        self.low_V = max(1, min(self.high_V-1, self.low_V))

    def on_high_V_thresh_trackbar(self, val):
        self.high_V = val
        self.high_V = max(self.high_V, self.low_V+1)

    def save(self):
        self.settings['objects'][self.name]['threshold'] = [
            self.low_H, self.high_H, self.low_S, self.high_S, self.low_V, self.high_V]
        self.settings.save()

    def get_angle(self):
        angle = math.atan2(self.center[1] - self.mirror_center[1], self.center[0] - self.mirror_center[0])
        angle = (270 - math.degrees(angle) + 360) % 360
        return angle
    
    def get_dist(self):
        return math.sqrt((self.center[0] - self.mirror_center[0])**2 + (self.center[1] - self.mirror_center[1])**2)

    def draw(self, frame):
        pass


class Ball(TrackingObject):
    def __init__(self, settings):
        super().__init__(settings, 'ball')
        self.radius = None
        self.min_area = 100

    def find(self, frame_HSV):
        self.frame_threshold = cv.inRange(
            frame_HSV, (self.low_H, self.low_S, self.low_V), (self.high_H, self.high_S, self.high_V))
        contours, hierarchy = cv.findContours(self.frame_threshold, 1, 2)

        self.visible = False
        if len(contours):
            cnt = max(contours, key=lambda x: cv.contourArea(x))
            M = cv.moments(cnt)

            if cv.contourArea(cnt) > self.min_area:
                self.visible = True
                self.center, self.radius = cv.minEnclosingCircle(cnt)
                self.angle = self.get_angle()
                self.dist = self.get_dist()

    def draw(self, frame):
        if self.visible:
            center = (int(self.center[0]), int(self.center[1]))
            radius = int(self.radius)
            cv.circle(frame, center, radius, (0, 0, 255), 5)
            cv.line(frame, self.mirror_center, center, (0, 0, 255), 5)


class Goal(TrackingObject):
    def __init__(self, settings, name):
        super().__init__(settings, name)
        self.rect: cv.RotatedRect = None
        self.size = None
        self.rotation = None
        self.min_area = 100

    def find(self, frame_HSV):
        self.frame_threshold = cv.inRange(
            frame_HSV, (self.low_H, self.low_S, self.low_V), (self.high_H, self.high_S, self.high_V))
        contours, hierarchy = cv.findContours(self.frame_threshold, 1, 2)

        if len(contours):
            cnt = max(contours, key=lambda x: cv.contourArea(x))
            M = cv.moments(cnt)

            if cv.contourArea(cnt) > self.min_area:
                self.visible = True
                self.rect = cv.minAreaRect(cnt)
                self.center, self.size, self.rotation = self.rect
                self.angle = self.get_angle()
                self.dist = self.get_dist()
        else:
            self.visible = False

    def draw(self, frame, color):
        if self.visible:
            center = (int(self.center[0]), int(self.center[1]))
            box = cv.boxPoints(self.rect)
            box = np.array(box).astype(int)
            cv.drawContours(frame, [box], 0, color, 5)
            cv.line(frame, self.mirror_center, center, color, 5)


class YellowGoal(Goal):
    def __init__(self, settings):
        super().__init__(settings, 'yellow goal')
        self.rect = None

    def draw(self, frame):
        super().draw(frame, (0, 255, 255))


class BlueGoal(Goal):
    def __init__(self, settings):
        super().__init__(settings, 'blue goal')
        self.rect = None

    def draw(self, frame):
        super().draw(frame, (255, 0, 0))

