#include "tracking/ball.h"

#include <vector>

using namespace std;

Ball::Ball(const vector<int> &hsv_min, const vector<int> &hsv_max)
    : Object(hsv_min, hsv_max) {}
Ball::~Ball() {};

void Ball::find(cv::Mat frame) {
    cv::Mat range;
    cv::inRange(frame, hsv_min, hsv_max, range);
    vector<cv::Mat> contours;
    cv::findContours(range, contours, 1, 2);
    if (contours.empty()) {
        visible = false;
        return;
    }
    int mx_area = 0;
    int j = -1;
    for (int i = 0; i < contours.size(); ++i) {
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