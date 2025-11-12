#include "tracking/object.h"

#include <spdlog/spdlog.h>

#include <vector>

#include "robot.h"
#include "utils/config.h"

using namespace std;

Object::~Object() {};

double Object::get_pixels_dist() {
  int radius = config.tracking.radius;
  Vec mirror_center = {radius, radius};
  return (center - mirror_center).len();
}

Object::Object(const vector<int>& hsv_min, const vector<int>& hsv_max,
               bool is_setup, int min_area_)
    : setup_mode(is_setup), min_area(min_area_) {
  h_min = hsv_min[0];
  s_min = hsv_min[1];
  v_min = hsv_min[2];
  h_max = hsv_max[0];
  s_max = hsv_max[1];
  v_max = hsv_max[2];
  if (config.tracking.preview.enabled && setup_mode) {
    string window_name = config.tracking.preview.window_name;
    cv::createTrackbar("H min", window_name, &h_min, 179);
    cv::createTrackbar("H max", window_name, &h_max, 179);
    cv::createTrackbar("S min", window_name, &s_min, 255);
    cv::createTrackbar("S max", window_name, &s_max, 255);
    cv::createTrackbar("V min", window_name, &v_min, 255);
    cv::createTrackbar("V max", window_name, &v_max, 255);
  }
}

void Object::find(const cv::Mat& frame) {
  cv::inRange(frame, cv::Scalar(h_min, s_min, v_min),
              cv::Scalar(h_max, s_max, v_max), mask);

  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 3);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 3);

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

  if (mx_area < min_area) {
    visible = false;
    return;
  }

  visible = true;
  cv::Point2f point_center;
  float float_radius;
  cv::minEnclosingCircle(contours[j], point_center, float_radius);
  center = point_center;
  radius = float_radius;
  camera_point =
      (Vec)center - Vec{config.tracking.radius, config.tracking.radius};

  relative_angle = normalize_angle(-1 * camera_point.raw_angle() + M_PI);
}

void Object::draw(cv::Mat& frame) {
  if (setup_mode) {
    cv::Mat preview;
    cv::bitwise_and(frame, frame, preview, mask);

    // Чтобы было лучше видно — можно поверх оригинала полупрозрачно выделить
    // цветом
    cv::Mat colored;
    cv::cvtColor(mask, colored, cv::COLOR_GRAY2BGR);
    colored.setTo(cv::Scalar(255, 255, 255), mask);  // красным

    cv::addWeighted(frame, 1, colored, 0.8, 0, frame);
    cv::circle(frame, {frame.size().width / 2, frame.size().height / 2}, 5,
               {0, 0, 255}, 10);

    return;
  }
  if (!visible) {
    return;
  }
  cv::circle(frame, center, radius, 100, 10);
}

double Object::get_cm() {
  if (override_dist != -1) {
    return override_dist;
  }
  double old_pixels =
      get_pixels_dist() * 800 * config.tracking.k / config.tracking.width;
  return 7612.57165 / (392.22648 - old_pixels) - 17.45807;
}

void Object::compute_field_position(const Robot& robot) {
  double ball_angle = robot.field_angle + relative_angle;
  Vec offset{-1 * sin(ball_angle) * get_cm(), cos(ball_angle) * get_cm()};
  field_position = robot.position + offset;
  // if (field_position.x < -20 || field_position.x > 182 + 20 ||
  //     field_position.y < -20 || field_position.y > 243 + 40) {
  //   spdlog::warn("Wrong object position: {} {}", field_position.x,
  //                field_position.y);
  //   visible = false;
  // }
}
