#include "tracking/ball.h"

#include <vector>

#include "utils/config.h"

using namespace std;

Ball::Ball(const vector<int> &hsv_min, const vector<int> &hsv_max,
           bool is_setup)
    : Object(hsv_min, hsv_max), setup_mode(is_setup) {
  if (config["tracking"]["preview"]["enabled"].GetBool() &&
      config["tracking"]["preview"]["setup"].GetBool()) {
    string window_name =
        config["tracking"]["preview"]["window_name"].GetString();
    cv::createTrackbar("H min", window_name, &h_min, 179);
    cv::createTrackbar("H max", window_name, &h_max, 179);
    cv::createTrackbar("S min", window_name, &s_min, 255);
    cv::createTrackbar("S max", window_name, &s_max, 255);
    cv::createTrackbar("V min", window_name, &v_min, 255);
    cv::createTrackbar("V max", window_name, &v_max, 255);
  }
}
Ball::~Ball() {};

void Ball::find(const cv::Mat &frame) {
  cv::inRange(frame, cv::Scalar(h_min, s_min, v_min),
              cv::Scalar(h_max, s_max, v_max), mask);

  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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
  camera_point = (Vec)center - Vec{config["tracking"]["radius"].GetInt(),
                                   config["tracking"]["radius"].GetInt()};

  relative_angle = normalize_angle(-1 * camera_point.raw_angle() + M_PI);
}

void Ball::draw(cv::Mat &frame) {
  if (setup_mode) {
    cv::Mat preview;
    cv::bitwise_and(frame, frame, preview, mask);

    // Чтобы было лучше видно — можно поверх оригинала полупрозрачно выделить
    // цветом
    cv::Mat colored;
    cv::cvtColor(mask, colored, cv::COLOR_GRAY2BGR);
    colored.setTo(cv::Scalar(0, 255, 0), mask);  // красным

    cv::addWeighted(frame, 1, colored, 0.8, 0, frame);

    return;
  }
  if (!visible) {
    cout << "not visible" << endl;
    return;
  }
  cv::circle(frame, center, radius, 100, 10);
}

float Ball::get_cm() {
  if (override_dist != -1) {
    return override_dist;
  }
  double old_pixels =
      get_pixels_dist() * 972 / config["tracking"]["width"].GetInt();
  return 7612.57165 / (392.22648 - old_pixels) - 17.45807;
}
