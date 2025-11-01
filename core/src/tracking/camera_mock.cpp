#include "tracking/camera.h"

struct Camera::Impl {};

Camera::Camera(Object& b) : ball(b) {};
Camera::~Camera() = default;

void Camera::start() {}

void Camera::show_preview() {}