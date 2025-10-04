#include "tracking/camera.h"

struct Camera::Impl {};

Camera::Camera(Ball &b) : ball(b) {};
Camera::~Camera() = default;

void Camera::start() {}

void Camera::show_preview() {}