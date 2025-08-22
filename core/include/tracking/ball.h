#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

#include "tracking/object.h"

constexpr int MIN_BALL_AREA = 10;

class Ball : Object {
   public:
    bool visible = false;
    double radius;

    Ball(std::vector<int>, std::vector<int>);
    virtual ~Ball();
    void find(cv::Mat frame);
    void draw(cv::Mat frame);
};