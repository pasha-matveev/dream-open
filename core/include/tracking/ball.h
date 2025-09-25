#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "tracking/object.h"
#include "utils/vec.h"

constexpr int MIN_BALL_AREA = 10;

class Ball : Object {
   public:
    bool visible = false;
    double radius;

    Vec point;

    Ball() = default;
    Ball(const std::vector<int> &, const std::vector<int> &);
    virtual ~Ball();
    void find(const cv::Mat &frame);
    void draw(cv::Mat frame);
    float get_cm();
    float angle();
};