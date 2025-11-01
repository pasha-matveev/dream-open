#pragma once

#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tracking/object.h"

class Camera {
 public:
  std::thread camera_thread;
  struct Impl;
  std::unique_ptr<Impl> impl;

  Camera(Object& ball_, Object& goal_);
  ~Camera();

  Object& ball;
  Object& goal;

  void start();
  void show_preview();
};