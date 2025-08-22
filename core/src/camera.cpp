#include "camera.h"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

using namespace std;

Camera::Camera(Ball &b, bool preview) : ball(b), has_preview(preview) {
    mask = cv::Mat(cv::Size(RADIUS * 2, RADIUS * 2), 0);
    circle(mask, {RADIUS, RADIUS}, RADIUS, 255, -1);
    circle(mask, {RADIUS, RADIUS}, DISABLED_RADIUS, 0, -1);
}

void Camera::capture() {
    cv::Mat temp;
    auto grabbed = video.read(temp);
    if (!grabbed) {
        return;
    }
    resize(temp, temp, cv::Size(RADIUS * 2, RADIUS * 2));
    frame = cv::Mat();
    bitwise_and(temp, temp, frame, mask);
    cv::cvtColor(frame, hsv_frame, cv::COLOR_RGB2HSV);
}

void Camera::analyze() { ball.find(hsv_frame); }

void Camera::draw() {
    preview_image = frame;
    ball.draw(preview_image);
}

void Camera::show_preview() {
    if (preview_image.empty()) return;
    imshow(WINDOW_NAME, preview_image);
}

void Camera::cycle() {
    int delay = 1000 / VIDEO_FPS / 2;
    while (true) {
        cout << "Run camera cycle\n";
        capture();
        analyze();
        if (has_preview) {
            draw();
        }
        this_thread::sleep_for(chrono::milliseconds(delay));
    }
}

void start_camera_cycle(Camera *camera) { camera->cycle(); }

void Camera::start() {
    for (int time = 10; time > 0; --time) {
        video = cv::VideoCapture(0);
        video.set(cv::CAP_PROP_FPS, VIDEO_FPS);
        video.set(cv::CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH);
        video.set(cv::CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT);
        if (video.isOpened()) {
            cout << "Camera connected\n";
            break;
        }
        this_thread::sleep_for(chrono::seconds(1));
    }
    if (!video.isOpened()) {
        cout << "Can not connect to camera\n";
        throw runtime_error("Can not connect to camera");
    }
    thread camera_thread(start_camera_cycle, this);
    camera_thread.detach();
}