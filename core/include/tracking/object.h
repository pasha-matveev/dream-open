#pragma once

#include <vector>

#include "utils/vec.h"

class Robot;

class Object {
  int min_area;

 public:
  bool setup_mode;

  double override_dist = -1;
  cv::Mat mask;
  bool visible = false;
  double radius;
  Vec camera_point;
  float relative_angle;
  Vec field_position{0, 0};

  int h_min, s_min, v_min;
  int h_max, s_max, v_max;
  Vec center;
  double angle;
  double dist;

  Object() = default;
  Object(const std::vector<int>&, const std::vector<int>&, bool, int);

  void find(const cv::Mat& frame);
  void draw(cv::Mat& frame);
  void compute_field_position(const Robot& robot);

  float get_pixels_dist();
  float get_cm();

  virtual ~Object();
};