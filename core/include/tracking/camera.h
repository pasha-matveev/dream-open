#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

class Object;

class Camera {
 public:
  std::thread camera_thread;
  struct Impl;
  std::unique_ptr<Impl> impl;

  Camera(Object& ball_, Object& goal_, Object& own_goal_);
  ~Camera();

  Object& ball;
  Object& goal;
  Object& own_goal;

  void start();
  void stop();
  void show_preview();
  bool new_data();

  // Углы (в кадре робота, радианы) пересечений кольца сэмплирования с чёрной
  // линией, найденные в последнем кадре. Снимок под mutex — безопасно звать из
  // потока стратегии. Пусто, если линия не видна или режим не "line".
  std::vector<double> get_line_candidates();
};
