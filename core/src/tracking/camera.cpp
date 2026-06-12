#include "tracking/camera.h"

#include <spdlog/spdlog.h>
#include <sys/mman.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

#include "tracking/object.h"
#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/line.h"
#include "config/tracking.h"
#include "utils/geo/vec.h"
#include "utils/millis.h"

using namespace std;
using namespace chrono;

long long millis();

namespace {

// Сэмплит кольцо радиуса ring_radius_px вокруг центра изображения и возвращает
// углы (в кадре робота, рад) середин чёрных дуг — точек, где кольцо пересекает
// линию. «Чёрный» сэмпл: средний V по маленькому радиальному патчу <=
// v_threshold (денойз против одиночного шума и алиасинга тонкой линии). Дуги
// фильтруются по [min_arc_deg, max_arc_deg]: узкие — шум, широкие — тело
// робота/тень. Конвенция угла та же, что в Object::find:
//   θ = normalize_angle(M_PI - φ),  φ = raw_angle = atan2(x, y).
std::vector<double> sample_line_ring(const cv::Mat& hsv) {
  std::vector<double> out;
  const cfg::Line& lc = *config->strategy->line;
  const int radius = config->tracking->radius;
  const int R = lc.ring_radius_px;
  const int N = lc.samples;
  if (N <= 0 || R <= 0) return out;

  std::vector<char> black(N, 0);
  for (int i = 0; i < N; ++i) {
    const double phi = 2.0 * M_PI * i / N;
    const double s = std::sin(phi), c = std::cos(phi);
    int sum = 0, cnt = 0;
    for (int dr = -1; dr <= 1; ++dr) {
      const double rr = R + dr;
      const int px = (int)std::lround(radius + rr * s);
      const int py = (int)std::lround(radius + rr * c);
      if (px < 0 || py < 0 || px >= hsv.cols || py >= hsv.rows) continue;
      sum += hsv.at<cv::Vec3b>(py, px)[2];
      ++cnt;
    }
    if (cnt > 0 && sum / cnt <= lc.v_threshold) black[i] = 1;
  }

  // Старт скана с не-чёрного сэмпла, чтобы дуга на стыке 0/2π считалась один раз.
  int start = -1;
  for (int i = 0; i < N; ++i)
    if (!black[i]) {
      start = i;
      break;
    }
  if (start < 0) return out;  // всё кольцо чёрное → линии нет

  const double deg_per = 360.0 / N;
  int run_len = 0, run_start = 0;
  for (int k = 0; k <= N; ++k) {
    const bool b = (k < N) && black[(start + k) % N];
    if (b) {
      if (run_len == 0) run_start = k;
      ++run_len;
    } else if (run_len > 0) {
      const double width_deg = run_len * deg_per;
      if (width_deg >= lc.min_arc_deg && width_deg <= lc.max_arc_deg) {
        const double mid_k = run_start + (run_len - 1) / 2.0;
        const double phi = 2.0 * M_PI * (start + mid_k) / N;
        out.push_back(normalize_angle(M_PI - phi));
      }
      run_len = 0;
    }
  }
  return out;
}

}  // namespace

struct Camera::Impl {
 public:
  cv::Mat mask;
  cv::Mat frame;
  cv::Mat hsv_frame;
  cv::Mat preview_image;
  Object& ball;
  Object& goal;
  Object& own_goal;
  std::vector<double> line_candidates;
  std::mutex line_mtx;
  bool preview_ready = false;
  std::atomic<bool> running{true};
  std::atomic<bool> new_data{false};
  // Ставится в самом конце start(). Позволяет Camera::stop() отличить штатно
  // завершившийся поток от зависшего (например, в cam.startVideo()).
  std::atomic<bool> thread_exited{false};

  void start();
  void analyze();
  void draw();
  void requestComplete();
  void show_preview();
  void request_stop();

  Impl(Object& ball, Object& goal, Object& own_goal);
  ~Impl() = default;
};

Camera::Impl::Impl(Object& ball_reference, Object& goal_, Object& own_goal_)
    : ball(ball_reference), goal(goal_), own_goal(own_goal_) {
  const int radius = config->tracking->radius,
            disabled_radius = config->tracking->disabled_radius;
  mask = cv::Mat(cv::Size(radius * 2, radius * 2), 0);
  circle(mask, {radius, radius}, 1000, 0, -1);
  circle(mask, {radius, radius}, radius, 255, -1);
  circle(mask, {radius, radius}, disabled_radius, 0, -1);
}

Camera::Camera(Object& b, Object& goal_, Object& own_goal_)
    : ball(b), goal(goal_), own_goal(own_goal_) {
  impl = make_unique<Impl>(ball, goal, own_goal);
}

void Camera::Impl::analyze() {
  ball.find(hsv_frame);
  goal.find(hsv_frame);
  own_goal.find(hsv_frame);

  // Кольцевой детектор линии — только в режиме "line", иначе лишняя работа.
  if (config->strategy->role == "line") {
    std::vector<double> cand = sample_line_ring(hsv_frame);
    std::lock_guard<std::mutex> lock(line_mtx);
    line_candidates = std::move(cand);
  }
}

void Camera::Impl::draw() {
  preview_ready = false;
  preview_image = frame;
  ball.draw(preview_image);
  goal.draw(preview_image);
  own_goal.draw(preview_image);

  // Оверлей линии для калибровки: кольцо + найденные пересечения (θ).
  if (config->strategy->role == "line") {
    const int radius = config->tracking->radius;
    const int R = config->strategy->line->ring_radius_px;
    const cv::Point center{radius, radius};
    cv::circle(preview_image, center, R, cv::Scalar(0, 255, 0), 2);
    std::vector<double> cand;
    {
      std::lock_guard<std::mutex> lock(line_mtx);
      cand = line_candidates;
    }
    for (double theta : cand) {
      const double phi = M_PI - theta;  // обратное к θ = M_PI - φ
      const cv::Point p{(int)std::lround(radius + R * std::sin(phi)),
                        (int)std::lround(radius + R * std::cos(phi))};
      cv::line(preview_image, center, p, cv::Scalar(255, 0, 0), 2);
      cv::circle(preview_image, p, 8, cv::Scalar(255, 0, 0), -1);
    }
  }
  preview_ready = true;
}

void Camera::show_preview() { impl->show_preview(); }

void Camera::Impl::show_preview() {
  if (!preview_ready) {
    return;
  }
  cv::Mat small_preview;
  cv::resize(preview_image, small_preview, cv::Size(440, 440));
  imshow("Camera", small_preview);
}

void Camera::Impl::request_stop() { running.store(false); }

void Camera::Impl::start() {
  lccv::PiCamera cam;
  cam.options->video_width = config->tracking->width;
  cam.options->video_height = config->tracking->height;
  cam.options->framerate = config->tracking->fps;
  cam.options->verbose = true;
  cam.options->brightness = config->tracking->brightness;
  cam.startVideo();

  long long start_time = millis();
  int cnt = 0;

  while (running.load()) {
    if (!cam.getVideoFrame(frame, 1000)) {
      if (!running.load()) {
        break;
      }
      spdlog::warn("Camera frame timeout");
    } else {
      if (frame.size().width != config->tracking->width) {
        spdlog::error("Bad frame. Resolution: {} {}", frame.size().width,
                      frame.size().height);
        continue;
      }
      const int radius = config->tracking->radius;
      const int center_x = config->tracking->center->x;
      const int center_y = config->tracking->center->y;
      const int width = config->tracking->width;
      const int height = config->tracking->height;
      int x1 = center_x - radius;
      int y1 = center_y - radius;
      cv::Mat source = frame(cv::Rect(x1, y1, radius * 2, radius * 2));
      cv::bitwise_and(source, source, frame, mask);
      cv::cvtColor(frame, hsv_frame, cv::COLOR_RGB2HSV);

      analyze();
      if (config->tracking->preview_enabled) {
        draw();
      }

      ++cnt;
      long long elapsed = millis() - start_time;
      if (elapsed >= 1000) {
        spdlog::info("FPS: {}", cnt);
        cnt = 0;
        start_time = millis();
      }

      new_data.store(true, std::memory_order_release);
    }
  }
  cam.stopVideo();
  thread_exited.store(true, std::memory_order_release);
}

void Camera::start() {
  if (camera_thread.joinable()) {
    spdlog::warn("Camera already running; ignoring start request");
    return;
  }
  if (impl) {
    impl->running.store(true);
    impl->new_data.store(false, std::memory_order_release);
    impl->thread_exited.store(false, std::memory_order_release);
  }
  camera_thread = thread(&Impl::start, &(*impl));
}

void Camera::stop() {
  if (impl) {
    impl->request_stop();
  }
  if (camera_thread.joinable()) {
    // Поток камеры обычно выходит за ~1с (getVideoFrame с таймаутом 1000мс
    // видит running=false). Но если он намертво завис в startVideo(),
    // безусловный join() заблокировал бы очистку навсегда. Ждём флаг с
    // дедлайном; не дождались — detach (поток утечёт, но процесс всё равно
    // завершается, ОС освободит устройство камеры).
    const auto deadline = steady_clock::now() + seconds(3);
    while (impl && !impl->thread_exited.load(std::memory_order_acquire) &&
           steady_clock::now() < deadline) {
      this_thread::sleep_for(milliseconds(50));
    }
    if (impl && impl->thread_exited.load(std::memory_order_acquire)) {
      camera_thread.join();
    } else {
      spdlog::warn("Camera thread stuck; detaching");
      camera_thread.detach();
      // Detached-поток продолжает держать указатель на Impl. Намеренно НЕ
      // освобождаем Impl: release() обнуляет unique_ptr без delete, иначе
      // ~Camera освободил бы память под живым потоком (use-after-free).
      // Impl утечёт, но процесс всё равно завершается — ОС реклеймит.
      impl.release();
    }
  }
}

bool Camera::new_data() {
  if (!impl) {
    return false;
  }
  return impl->new_data.exchange(false, std::memory_order_acq_rel);
}

std::vector<double> Camera::get_line_candidates() {
  if (!impl) {
    return {};
  }
  std::lock_guard<std::mutex> lock(impl->line_mtx);
  return impl->line_candidates;
}

Camera::~Camera() { stop(); }
