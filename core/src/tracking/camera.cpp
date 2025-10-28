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

// Конвертация из буфера в матрицу
struct MappedBuffer {
  void* memory;
  size_t length;
};

MappedBuffer mapBuffer(const libcamera::FrameBuffer* buffer) {
  const libcamera::FrameBuffer::Plane& plane = buffer->planes().front();
  int fd = plane.fd.get();
  size_t length = plane.length;
  off_t offset = plane.offset;

  void* memory =
      mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
  if (memory == MAP_FAILED) {
    perror("mmap");
    throw std::runtime_error("mmap failed");
  }

  return {memory, length};
}

cv::Mat toMat(const libcamera::FrameBuffer* buffer) {
  auto mapped = mapBuffer(buffer);
  int width = config.tracking.width;
  int height = config.tracking.height;
  size_t min_stride = static_cast<size_t>(width) * 3;
  size_t stride = min_stride;
  if (height > 0) {
    size_t h = static_cast<size_t>(height);
    const auto& plane = buffer->planes().front();
    if (plane.length >= h) {
      size_t candidate = plane.length / h;
      if (candidate >= min_stride && candidate > stride) {
        stride = candidate;
      }
    }
    const libcamera::FrameMetadata& metadata = buffer->metadata();
    const auto& planes_md = metadata.planes();
    if (!planes_md.empty() && planes_md[0].bytesused >= min_stride * h) {
      size_t candidate = planes_md[0].bytesused / h;
      if (candidate >= min_stride && candidate > stride) {
        stride = candidate;
      }
    }
  }

  cv::Mat img(height, width, CV_8UC3, mapped.memory, stride);

  cv::Mat copy = img.clone();

  munmap(mapped.memory, mapped.length);

  return copy;
}

struct Camera::Impl {
  cv::Mat mask;
  cv::Mat frame;
  cv::Mat hsv_frame;
  cv::Mat preview_image;
  Ball& ball;
  bool preview_ready = false;

  std::unique_ptr<libcamera::CameraManager> cm;
  std::shared_ptr<libcamera::Camera> lcamera;
  std::vector<std::unique_ptr<libcamera::Request>> requests;
  libcamera::StreamConfiguration* streamConfig;

  void start();
  void analyze();
  void draw();
  void requestComplete(libcamera::Request* request);
  void show_preview();

  Impl(Ball& ball);
  ~Impl() = default;
};

Camera::Impl::Impl(Ball& ball_reference) : ball(ball_reference) {
  const int radius = config.tracking.radius,
            disabled_radius = config.tracking.disabled_radius;
  mask = cv::Mat(cv::Size(radius * 2, radius * 2), 0);
  circle(mask, {radius, radius}, 1000, 0, -1);
  circle(mask, {radius, radius}, radius, 255, -1);
  circle(mask, {radius, radius}, disabled_radius, 0, -1);

  cm = make_unique<libcamera::CameraManager>();
}

Camera::Camera(Ball& b) : ball(b) { impl = make_unique<Impl>(ball); }
Camera::~Camera() = default;

void Camera::Impl::analyze() { ball.find(hsv_frame); }

void Camera::Impl::draw() {
  preview_ready = false;
  preview_image = frame;
  ball.draw(preview_image);
  preview_ready = true;
}

void Camera::show_preview() { impl->show_preview(); }

void Camera::Impl::show_preview() {
  if (!preview_ready) {
    return;
  }
  cv::Mat small_preview;
  cv::resize(preview_image, small_preview, cv::Size(440, 440));
  imshow(config.tracking.preview.window_name, small_preview);
}

int f = 0;
auto st = clock();

void Camera::Impl::requestComplete(libcamera::Request* request) {
  if (request->status() == libcamera::Request::RequestCancelled) return;
  const std::map<const libcamera::Stream*, libcamera::FrameBuffer*>& buffers =
      request->buffers();
  assert(buffers.size() == 1);
  for (const auto [_, buffer] : buffers) {
    frame = toMat(buffer);
  }
  request->reuse(libcamera::Request::ReuseBuffers);
  lcamera->queueRequest(request);

  const int radius = config.tracking.radius;
  const int center_x = config.tracking.center.x;
  const int center_y = config.tracking.center.y;
  const int width = config.tracking.width;
  const int height = config.tracking.height;
  int x1 = center_x - radius;
  int y1 = center_y - radius;
  cv::Mat source = frame(cv::Rect(x1, y1, radius * 2, radius * 2));
  cv::bitwise_and(source, source, frame, mask);
  cv::cvtColor(frame, hsv_frame, cv::COLOR_RGB2HSV);

  analyze();
  draw();

  ++f;

  auto t = (clock() - st) * 1000 / CLOCKS_PER_SEC;

  if (t >= 1000) {
    spdlog::info("FPS: {}", f);
    f = 0;
    st = clock();
  }
};

void Camera::Impl::start() {
  // Подключение камеры
  cm->start();
  auto cameras = cm->cameras();
  if (cameras.empty()) {
    throw runtime_error("No camera found");
  }
  string camera_id = cameras[0]->id();
  lcamera = cm->get(camera_id);
  lcamera->acquire();
  lcamera->requestCompleted.connect(this, &Camera::Impl::requestComplete);
  // Настройки
  unique_ptr<libcamera::CameraConfiguration> camera_config =
      lcamera->generateConfiguration({libcamera::StreamRole::Viewfinder});
  auto& streamConfig = camera_config->at(0);
  streamConfig.size.width = config.tracking.width;
  streamConfig.size.height = config.tracking.height;
  streamConfig.pixelFormat = libcamera::formats::RGB888;
  camera_config->validate();
  spdlog::info("Camera configuration: {}", streamConfig.toString());
  int ret = lcamera->configure(camera_config.get());
  if (ret < 0) {
    throw runtime_error("Failed to configure camera");
  }

  // Выделение памяти
  libcamera::Stream* stream = streamConfig.stream();
  libcamera::FrameBufferAllocator* allocator =
      new libcamera::FrameBufferAllocator(lcamera);

  ret = allocator->allocate(stream);
  if (ret < 0) {
    throw runtime_error("Can't allocate buffers");
  }

  size_t allocated = allocator->buffers(streamConfig.stream()).size();

  const vector<unique_ptr<libcamera::FrameBuffer>>& buffers =
      allocator->buffers(stream);

  // Создание запросов к камере
  for (unsigned int i = 0; i < buffers.size(); ++i) {
    unique_ptr<libcamera::Request> request = lcamera->createRequest();
    if (!request) {
      throw runtime_error("Can't create request");
    }

    const unique_ptr<libcamera::FrameBuffer>& buffer = buffers[i];
    int res = request->addBuffer(stream, buffer.get());
    if (res < 0) {
      throw runtime_error("Can't set buffer for request");
    }

    int duration = (1000 / config.tracking.fps) * 1000;
    auto& controls = request->controls();
    controls.set(libcamera::controls::AeEnable, false);
    controls.set(libcamera::controls::ExposureTime, duration * 0.9);
    controls.set(libcamera::controls::AnalogueGain,
                 config.tracking.brightness);
    controls.set(libcamera::controls::FrameDurationLimits,
                 libcamera::Span<const int64_t, 2>({duration, duration}));

    requests.push_back(move(request));
  }

  lcamera->start();
  for (unique_ptr<libcamera::Request>& request : requests) {
    lcamera->queueRequest(request.get());
  }
}

void Camera::start() { impl->start(); }
