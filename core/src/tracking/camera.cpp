#include "tracking/camera.h"

#include <assert.h>
#include <libcamera/libcamera.h>
#include <spdlog/spdlog.h>
#include <sys/mman.h>

#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "utils/config.h"

using namespace std;

struct Camera::Impl {
    cv::Mat mask;
    cv::Mat frame;
    cv::Mat hsv_frame;
    cv::Mat preview_image;
    Ball ball;
    std::unique_ptr<libcamera::CameraManager> cm;
    std::shared_ptr<libcamera::Camera> lcamera;
    std::vector<std::unique_ptr<libcamera::Request>> requests;
    libcamera::StreamConfiguration *streamConfig;

    void analyze();
    void draw();
    void requestComplete(libcamera::Request *request);
}

// Конвертация из буфера в матрицу
struct MappedBuffer {
    void *memory;
    size_t length;
};

MappedBuffer mapBuffer(const libcamera::FrameBuffer *buffer) {
    const libcamera::FrameBuffer::Plane &plane = buffer->planes().front();
    int fd = plane.fd.get();
    size_t length = plane.length;

    void *memory =
        mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (memory == MAP_FAILED) {
        perror("mmap");
        throw std::runtime_error("mmap failed");
    }

    return {memory, length};
}

cv::Mat toMat(const libcamera::FrameBuffer *buffer) {
    auto mapped = mapBuffer(buffer);
    int width = config["tracking"]["width"].GetInt();
    int height = config["tracking"]["height"].GetInt();

    cv::Mat img(height, width, CV_8UC3, mapped.memory);

    cv::Mat copy = img.clone();

    munmap(mapped.memory, mapped.length);

    return copy;
}

Camera::Camera()
    : ball(make_int_vector(config["tracking"]["ball"]["hsv_min"].GetArray()),
           make_int_vector(config["tracking"]["ball"]["hsv_max"].GetArray())) {
    const int radius = config["tracking"]["radius"].GetInt(),
              disabled_radius = config["tracking"]["disabled_radius"].GetInt();
    mask = cv::Mat(cv::Size(radius * 2, radius * 2), 0);
    circle(mask, {radius, radius}, radius, 255, -1);
    circle(mask, {radius, radius}, disabled_radius, 0, -1);

    cm = make_unique<libcamera::CameraManager>();
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

int f = 0;
auto st = clock();

void Camera::requestComplete(libcamera::Request *request) {
    if (request->status() == libcamera::Request::RequestCancelled) return;
    const std::map<const libcamera::Stream *, libcamera::FrameBuffer *>
        &buffers = request->buffers();
    assert(buffers.size() == 1);
    for (const auto [_, buffer] : buffers) {
        frame = toMat(buffer);
    }
    request->reuse(libcamera::Request::ReuseBuffers);
    lcamera->queueRequest(request);

    const int radius = config["tracking"]["radius"].GetInt();
    const int center_x = config["tracking"]["center"]["x"].GetInt();
    const int center_y = config["tracking"]["center"]["y"].GetInt();
    const int width = config["tracking"]["width"].GetInt();
    const int height = config["tracking"]["height"].GetInt();
    int x1 = center_x - radius;
    int y1 = center_y - radius;
    frame = frame(cv::Rect(x1, y1, radius * 2, radius * 2));
    bitwise_and(frame, frame, frame, mask);
    cv::cvtColor(frame, hsv_frame, cv::COLOR_RGB2HSV);

    analyze();
    draw();

    ++f;

    auto t = (clock() - st) * 1000 / CLOCKS_PER_SEC;

    if (t >= 1000) {
        cout << "FPS: " << f << endl;
        f = 0;
        st = clock();
    }
};

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
    lcamera->requestCompleted.connect(this, &Camera::requestComplete);
    // Настройки
    unique_ptr<libcamera::CameraConfiguration> camera_config =
        lcamera->generateConfiguration({libcamera::StreamRole::Viewfinder});
    auto &streamConfig = camera_config->at(0);
    streamConfig.size.width = config["tracking"]["width"].GetInt();
    streamConfig.size.height = config["tracking"]["height"].GetInt();
    streamConfig.pixelFormat = libcamera::formats::RGB888;
    camera_config->validate();
    spdlog::info("Camera configuration: {}", streamConfig.toString());
    int ret = lcamera->configure(camera_config.get());
    if (ret < 0) {
        throw runtime_error("Failed to configure camera");
    }

    // Выделение памяти
    libcamera::Stream *stream = streamConfig.stream();
    libcamera::FrameBufferAllocator *allocator =
        new libcamera::FrameBufferAllocator(lcamera);

    ret = allocator->allocate(stream);
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

        int duration = (1000 / config["tracking"]["fps"].GetInt()) * 1000;
        auto &controls = request->controls();
        controls.set(libcamera::controls::AeEnable, false);
        controls.set(libcamera::controls::ExposureTime, duration * 0.9);
        controls.set(libcamera::controls::AnalogueGain,
                     config["tracking"]["brightness"].GetFloat());
        controls.set(libcamera::controls::FrameDurationLimits,
                     libcamera::Span<const int64_t, 2>({duration, duration}));

        requests.push_back(move(request));
    }

    lcamera->start();
    for (unique_ptr<libcamera::Request> &request : requests) {
        lcamera->queueRequest(request.get());
    }
}