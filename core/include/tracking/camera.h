#pragma once

#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tracking/ball.h"

class Camera {
 public:
  std::thread camera_thread;
  struct Impl;
  std::unique_ptr<Impl> impl;

  Camera(Ball&);
  ~Camera();

  Ball& ball;

  void start();
  void show_preview();
};