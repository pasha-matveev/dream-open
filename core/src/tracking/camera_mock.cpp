#include "tracking/camera.h"

struct Camera::Impl {
  void request_stop() {}
};

Camera::Camera(Object& ball_, Object& goal_) : ball(ball_), goal(goal_) {};
Camera::~Camera() { stop(); }

void Camera::start() {}

void Camera::stop() {}

void Camera::show_preview() {}
