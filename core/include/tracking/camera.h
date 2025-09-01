#pragma once

#include <libcamera/libcamera.h>

#include <opencv2/opencv.hpp>

#include "tracking/ball.h"

class Camera {
   private:
    cv::Mat mask;
    cv::Mat temp;
    cv::Mat frame;
    cv::Mat hsv_frame;
    cv::Mat preview_image;
    cv::VideoCapture video;
    Ball &ball;
    bool has_preview;
    std::unique_ptr<libcamera::CameraManager> cm;
    std::shared_ptr<libcamera::Camera> lcamera;
    std::vector<std::unique_ptr<libcamera::Request>> requests;

    void capture();
    void analyze();
    void draw();
    void cycle();

   public:
    Camera(Ball &, bool);

    void start();
    void show_preview();

    friend void start_camera_cycle(Camera *);
};