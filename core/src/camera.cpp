#include "camera.h"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

using namespace std;
using namespace cv;

Camera::Camera() {
    mask = Mat(Size(RADIUS * 2, RADIUS * 2), 0);
    circle(mask, {RADIUS, RADIUS}, RADIUS, 255, -1);
    circle(mask, {RADIUS, RADIUS}, DISABLED_RADIUS, 0, -1);
}

void Camera::start() {
    for (int time = 10; time > 0; --time) {
        video = VideoCapture(0);
        video.set(CAP_PROP_FPS, VIDEO_FPS);
        video.set(CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH);
        video.set(CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT);
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
}

void Camera::capture() {
    Mat temp;
    auto grabbed = video.read(temp);
    if (!grabbed) {
        return;
    }
    resize(temp, temp, cv::Size(RADIUS * 2, RADIUS * 2));
    bitwise_and(temp, temp, image, mask);
    // image = temp;
}

void Camera::preview() { imshow(WINDOW_NAME, image); }