#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "tracking/object.h"
#include "utils/vec.h"

constexpr int MIN_BALL_AREA = 10;

class Ball : public Object {
 private:
  cv::Mat mask;
  bool setup_mode;

 public:
  bool visible = false;
  double radius;
  double override_dist = -1;

  Vec camera_point;
  float relative_angle;

  Ball() = default;
  Ball(const std::vector<int> &, const std::vector<int> &, bool);
  virtual ~Ball();
  void find(const cv::Mat &frame);
  void draw(cv::Mat &frame);
  float get_cm();
};