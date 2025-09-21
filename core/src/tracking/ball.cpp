#include "tracking/ball.h"

#include <vector>

using namespace std;

Ball::Ball(const vector<int> &hsv_min, const vector<int> &hsv_max)
    : Object(hsv_min, hsv_max) {}
Ball::~Ball() {};

void Ball::find(const cv::Mat &frame) {
    cv::Mat mask;
    cv::inRange(frame, hsv_min, hsv_max, mask);

    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        visible = false;
        return;
    }

    int j = -1;
    double mx_area = 0.0;
    for (int i = 0; i < (int)contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > mx_area) {
            mx_area = area;
            j = i;
        }
    }

    if (mx_area < MIN_BALL_AREA) {
        visible = false;
        return;
    }

    visible = true;
    cv::Point2f point_center;
    float float_radius;
    cv::minEnclosingCircle(contours[j], point_center, float_radius);
    center = point_center;
    radius = float_radius;
}

void Ball::draw(cv::Mat frame) {
    if (!visible) return;
    cv::circle(frame, center, radius, 100, 10);
}

float Ball::get_cm() {
    return 7612.57165 / (392.22648 - get_pixels_dist()) - 17.45807;
}