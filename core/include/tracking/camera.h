#pragma once

#include <memory>
#include <opencv2/opencv.hpp>

#include "tracking/ball.h"

class Camera {
   private:
    struct Impl;
    std::unique_ptr<Impl> impl;

   public:
    Camera();
    ~Camera();

    void start();
    void show_preview();
};