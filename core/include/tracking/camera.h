#pragma once

#include <atomic>
#include <memory>
#include <thread>

class Object;

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
  void stop();
  void show_preview();
  bool new_data();
};
