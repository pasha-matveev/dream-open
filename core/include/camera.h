#pragma once
#include <opencv2/opencv.hpp>
#include <string>

#include "tracking/ball.h"

constexpr int VIDEO_FPS = 30;
constexpr int VIDEO_WIDTH = 670 * 2;   // 1296;
constexpr int VIDEO_HEIGHT = 670 * 2;  // 972;
constexpr int RADIUS = 670;
constexpr int DISABLED_RADIUS = 10;
constexpr int CENTER_X = 645;
constexpr int CENTER_Y = 470;
const std::string WINDOW_NAME = "Camera";

class Camera {
   private:
    cv::Mat mask;
    cv::Mat frame;
    cv::Mat hsv_frame;
    cv::Mat preview_image;
    cv::VideoCapture video;
    Ball &ball;
    bool has_preview;

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