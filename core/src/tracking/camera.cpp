#include "tracking/camera.h"

#include <assert.h>

#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "utils/config.h"

using namespace std;

Camera::Camera(Ball &b, bool preview) : ball(b), has_preview(preview) {
    const int radius = config["tracking"]["radius"].GetInt(),
              disabled_radius = config["tracking"]["disabled_radius"].GetInt();
    mask = cv::Mat(cv::Size(radius * 2, radius * 2), 0);
    circle(mask, {radius, radius}, radius, 255, -1);
    circle(mask, {radius, radius}, disabled_radius, 0, -1);

    cm = make_unique<libcamera::CameraManager>();
}

void Camera::capture() {
    auto grabbed = video.read(temp);
    if (!grabbed) {
        return;
    }
    const int radius = config["tracking"]["radius"].GetInt();
    const int center_x = config["tracking"]["center"]["x"].GetInt();
    const int center_y = config["tracking"]["center"]["y"].GetInt();
    const int width = config["tracking"]["width"].GetInt();
    const int height = config["tracking"]["height"].GetInt();
    int x1 = center_x - radius;
    int y1 = center_y - radius;
    if (temp.size[0] != width || temp.size[1] != height) {
        cout << "Wrong frame dimensions: " << temp.size[0] << "x"
             << temp.size[1] << '\n';
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
    if (preview_image.empty()) {
        return;
    }
    imshow(config["tracking"]["preview"]["window_name"].GetString(),
           preview_image);
}

void Camera::cycle() {
    int delay = 1000 / config["tracking"]["fps"].GetInt() / 2;
    while (true) {
        long long cycle_start =
            chrono::steady_clock::now().time_since_epoch().count();
        capture();
        if (!frame.empty()) {
            analyze();
            if (has_preview) {
                draw();
            }
        }
        long long cycle_finish =
            chrono::steady_clock::now().time_since_epoch().count();
        long long required_sleep = delay - (cycle_finish - cycle_start);
        if (required_sleep > 0) {
            this_thread::sleep_for(chrono::milliseconds(required_sleep));
        }
    }
}

void start_camera_cycle(Camera *camera) { camera->cycle(); }

void requestComplete(libcamera::Request *request) {
    if (request->status() == libcamera::Request::RequestCancelled) return;
}

void Camera::start() {
    // Подключение камеры
    cm->start();
    auto cameras = cm->cameras();
    if (cameras.empty()) {
        throw runtime_error("No camera found");
    }
    string camera_id = cameras[0]->id();
    lcamera = cm->get(camera_id);
    lcamera->acquire();
    lcamera->requestCompleted.connect(requestComplete);
    // Настройки
    unique_ptr<libcamera::CameraConfiguration> camera_config =
        lcamera->generateConfiguration({libcamera::StreamRole::Viewfinder});
    auto &streamConfig = camera_config->at(0);
    streamConfig.size.width = config["tracking"]["width"].GetInt();
    streamConfig.size.height = config["tracking"]["height"].GetInt();
    camera_config->validate();
    cout << "Camera configuration is: " << streamConfig.toString() << endl;
    lcamera->configure(camera_config.get());

    // Выделение памяти
    libcamera::Stream *stream = streamConfig.stream();
    libcamera::FrameBufferAllocator *allocator =
        new libcamera::FrameBufferAllocator(lcamera);

    int ret = allocator->allocate(stream);
    if (ret < 0) {
        throw runtime_error("Can't allocate buffers");
    }

    size_t allocated = allocator->buffers(streamConfig.stream()).size();

    const vector<unique_ptr<libcamera::FrameBuffer>> &buffers =
        allocator->buffers(stream);

    for (unsigned int i = 0; i < buffers.size(); ++i) {
        unique_ptr<libcamera::Request> request = lcamera->createRequest();
        if (!request) {
            throw runtime_error("Can't create request");
        }

        const unique_ptr<libcamera::FrameBuffer> &buffer = buffers[i];
        int res = request->addBuffer(stream, buffer.get());
        if (res < 0) {
            throw runtime_error("Can't set buffer for request");
        }

        requests.push_back(move(request));
    }

    lcamera->start();
    for (unique_ptr<libcamera::Request> &request : requests) {
        lcamera->queueRequest(request.get());
    }
}