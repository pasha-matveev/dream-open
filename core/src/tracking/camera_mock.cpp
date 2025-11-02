#include "tracking/camera.h"

struct Camera::Impl {};

Camera::Camera(Object& ball_, Object& goal_) : ball(ball_), goal(goal_) {};
Camera::~Camera() = default;

void Camera::start() {}

void Camera::show_preview() {}