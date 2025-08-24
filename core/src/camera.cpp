#include "camera.h"

#include <assert.h>

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "config.h"

using namespace std;

Camera::Camera(Ball &b, bool preview) : ball(b), has_preview(preview) {
    const int radius = config["tracking"]["radius"],
              disabled_radius = config["tracking"]["disabled_radius"];
    mask = cv::Mat(cv::Size(radius * 2, radius * 2), 0);
    circle(mask, {radius, radius}, radius, 255, -1);
    circle(mask, {radius, radius}, disabled_radius, 0, -1);
}

void Camera::capture() {
    cv::Mat temp;
    auto grabbed = video.read(temp);
    if (!grabbed) {
        return;
    }
    const int radius = config["tracking"]["radius"];
    const int center_x = config["tracking"]["center"]["x"];
    const int center_y = config["tracking"]["center"]["y"];
    const int width = config["tracking"]["width"];
    const int height = config["tracking"]["height"];
    int x1 = center_x - radius;
    int y1 = center_y - radius;
    if (temp.size[0] != width) {
        cout << "Wrong frame width: " << temp.size[0] << '\n';
        exit(-1);
    }
    if (temp.size[1] != height) {
        cout << "Wrong frame height: " << temp.size[1] << '\n';
        exit(-1);
    }
    temp = temp(cv::Rect(x1, y1, radius * 2, radius * 2));
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
    imshow(config["tracking"]["window_name"], preview_image);
}

void Camera::cycle() {
    int delay = 1000 / (int)config["tracking"]["fps"] / 2;
    while (true) {
        cout << "Run camera cycle\n";
        capture();
        if (!frame.empty()) {
            analyze();
            if (has_preview) {
                draw();
            }
        }
        this_thread::sleep_for(chrono::milliseconds(delay));
    }
}

void start_camera_cycle(Camera *camera) { camera->cycle(); }

void Camera::start() {
    for (int time = 10; time > 0; --time) {
        video = cv::VideoCapture((int)config["tracking"]["camera_id"]);
        video.set(cv::CAP_PROP_FPS, (int)config["tracking"]["fps"]);
        video.set(cv::CAP_PROP_FRAME_WIDTH, (int)config["tracking"]["width"]);
        video.set(cv::CAP_PROP_FRAME_HEIGHT, (int)config["tracking"]["height"]);
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