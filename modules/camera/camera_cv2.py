from scripts.json_parser import JsonParser
from modules.camera.tracking_object import Ball, BlueGoal, YellowGoal
import cv2 as cv
import numpy as np


class CameraCV2:
    def __init__(self, args):
        self.args = args
        self.settings = JsonParser('modules/camera/calibration_data.json')
        self.res = self.settings['res']
        self.center = self.settings['center']
        self.inner_radius = self.settings['inner_radius']
        self.outer_radius = self.settings['outer_radius']

        self.mask = np.zeros((self.outer_radius * 2, self.outer_radius * 2), np.uint8)
        cv.circle(self.mask, (self.outer_radius, self.outer_radius), self.outer_radius, 255, -1)
        cv.circle(self.mask, (self.outer_radius, self.outer_radius), self.inner_radius, 0, -1)

        self.ball = Ball(self.settings)
        self.blue_goal = BlueGoal(self.settings)
        self.yellow_goal = YellowGoal(self.settings)

        if self.args.camera:
            self.start_preview()
        if self.args.ball:
            self.ball.start_preview_detection()
        if self.args.blue:
            self.blue_goal.start_preview_detection()
        if self.args.yellow:
            self.yellow_goal.start_preview_detection()
        
        self.start()
    
    def start(self):
        self.video = cv.VideoCapture(0)
        self.video.set(3, self.res[0])
        self.video.set(4, self.res[1])
        self.is_opened = self.video.isOpened()

    def get_frame(self):
        ret, self.frame = self.video.read()
        self.is_opened = ret

    def read(self):
        self.get_frame()

        self.frame = self.frame[max(self.center[1] - self.outer_radius, 0):min(self.center[1] + self.outer_radius, self.frame.shape[0]),
                                max(self.center[0] - self.outer_radius, 0):min(self.center[0] + self.outer_radius, self.frame.shape[1])]
        self.frame = cv.bitwise_and(self.frame, self.frame, mask=self.mask)
        self.frame_HSV = cv.cvtColor(self.frame, cv.COLOR_RGB2HSV)

        self.ball.find(self.frame_HSV)
        self.blue_goal.find(self.frame_HSV)
        self.yellow_goal.find(self.frame_HSV)

    def preview(self):
        if self.args.camera:
            self.ball.draw(self.frame)
            self.blue_goal.draw(self.frame)
            self.yellow_goal.draw(self.frame)
            cv.imshow('Camera', self.frame)
        if self.args.ball:
            self.ball.preview_detection()
        if self.args.blue:
            self.blue_goal.preview_detection()
        if self.args.yellow:
            self.yellow_goal.preview_detection()

    def start_preview(self):
        self.preview_name = 'Camera'
        cv.namedWindow(self.preview_name)
    
    def save(self):
        self.ball.save()
        self.blue_goal.save()
        self.yellow_goal.save()