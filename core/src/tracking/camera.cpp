#include "tracking/camera.h"

#include <assert.h>
#include <libcamera/libcamera.h>
#include <spdlog/spdlog.h>
#include <sys/mman.h>

#include <chrono>
#include <iostream>
#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "utils/config.h"

using namespace std;
using namespace chrono;

long long millis();

struct Camera::Impl {
 public:
  cv::Mat mask;
  cv::Mat frame;
  cv::Mat hsv_frame;
  cv::Mat preview_image;
  Ball& ball;
  bool preview_ready = false;

  void start();
  void analyze();
  void draw();
  void requestComplete();
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

void Camera::Impl::start() {
  lccv::PiCamera cam;
  cam.options->video_width = config.tracking.width;
  cam.options->video_height = config.tracking.height;
  cam.options->framerate = config.tracking.fps;
  cam.options->verbose = true;
  cam.options->brightness = config.tracking.brightness;
  cam.startVideo();

  long long start_time = millis();
  int cnt = 0;

  while (true) {
    if (!cam.getVideoFrame(frame, 100)) {
      cout << "Timeout" << endl;
    } else {
      if (frame.size().width != config.tracking.width) {
        cout << "Bad frame " << clock() << endl;
        continue;
      }
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
      if (config.tracking.preview.enabled) {
        draw();
      }

      ++cnt;
      long long elapsed = millis() - start_time;
      if (elapsed >= 1000) {
        spdlog::info("FPS: {}", cnt);
        cnt = 0;
        start_time = millis();
      }
    }
  }
}

void Camera::start() { camera_thread = thread(&Impl::start, &(*impl)); }
