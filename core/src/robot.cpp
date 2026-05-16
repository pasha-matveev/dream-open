#include "robot.h"

#include <libserial/SerialPort.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>

#include "config/config.h"
#include "config/gpio.h"
#include "config/lidar.h"
#include "config/serial.h"
#include "config/strategy.h"
#include "config/tracking.h"
#include "gpio/buttons.h"
#include "gpio/setup.h"
#include "media/img.h"
#include "utils/millis.h"

void Robot::read_from_arduino() {
  uart->write_data<char>('R');
  gyro_angle = normalize_angle2(-1 * uart->read_data<float>());

  raw_emitter = uart->read_data<int32_t>();
  if (raw_emitter < config->serial->emitter->threshold) {
    last_seen = millis();
  }
  emitter = (millis() - last_seen) < config->serial->emitter->optimist;

  kicker_charged = uart->read_data<bool>();
}

static bool written = false;

void Robot::write_to_arduino() {
  speed = actual_vel.len();
  direction = actual_vel.field_angle() - field_angle;
  uart->write_data<char>('W');
  uart->write_data<float>(
      normalize_angle2(-gyro_angle - normalize_angle2(direction)));
  uart->write_data<float>(speed);
  uart->write_data<float>(
      normalize_angle2(-gyro_angle - normalize_angle2(resolve_rotation())));
  uart->write_data<float>(rotation_limit);
  uart->write_data<int32_t>(dribbling);
  uart->write_data<int32_t>(kicker_force);
  uart->write_data<bool>(rgb_led);
  uart->write_data<bool>(state == RobotState::PAUSE);
}

void Robot::init_camera(Object& ball, Object& goal, Object& own_goal) {
  camera = new Camera(ball, goal, own_goal);
  camera->start();
}

void Robot::init_buzzer() { buzzer = new Buzzer(config->gpio->buzzer->pin); }

void Robot::beep() {
  if (buzzer) buzzer->beep();
}

void Robot::update_buzzer() {
  if (buzzer) buzzer->update();
}

void Robot::init_buttons() { setup_buttons(this); }

void Robot::init_uart() {
  uart = new UART();
  uart->connect();
  uart->wait_for_x();
}

void Robot::init_display() {
  string str_address = config->gpio->display->address;
  uint8_t address = stoi(str_address, nullptr, 16);
  display = new Display(config->gpio->display->device.c_str(), address);
  display->init();
  if (config->gpio->display->mode == "img") {
    display->draw_image(get_img(config->gpio->display->img));
  }
}

void Robot::init_lidar() {
  lidar = new Lidar();
  lidar->start();
  while (!lidar->received_data) {
    this_thread::sleep_for(chrono::milliseconds(50));
  }
}

void Robot::init_hardware(Object& ball, Object& goal, Object& own_goal) {
  if (config->tracking->enabled) {
    init_camera(ball, goal, own_goal);
  }
  spdlog::info("Camera ready");
  if (config->gpio->enabled) {
    setup_wiringpi();
    if (config->gpio->buzzer->enabled) {
      init_buzzer();
    }
    spdlog::info("Buzzer ready");
    if (config->gpio->buttons->enabled) {
      init_buttons();
    }
    spdlog::info("Buttons ready");
    if (config->gpio->display->enabled) {
      init_display();
    }
    spdlog::info("Camera ready");
  }
  spdlog::info("GPIO ready");
  if (config->serial->enabled) {
    init_uart();
    spdlog::info("Reading from arduino");
    read_from_arduino();
    spdlog::info("Got data from arduino");
    init_gyro();
  }
  spdlog::info("UART ready");
  if (config->lidar->enabled) {
    init_lidar();
  }
  spdlog::info("Lidar ready");
}

Robot::~Robot() {
  if (camera != nullptr) delete camera;
  if (buzzer != nullptr) delete buzzer;
  if (uart != nullptr) delete uart;
  if (display != nullptr) delete display;
  if (lidar != nullptr) delete lidar;
}

void Robot::init_gyro() {
  // top_angle = normalize_angle2(gyro_angle + M_PI / 2);
  top_angle = normalize_angle2(gyro_angle);
}

void Robot::calibrate() {
  top_angle = normalize_angle(gyro_angle - field_angle);
}

void Robot::calibrate(double measured_field_angle) {
  top_angle = normalize_angle(gyro_angle - measured_field_angle);
}

std::optional<LidarPose> Robot::compute_lidar() {
  if (!lidar) return std::nullopt;
  auto res = lidar->compute(*this);

  if (!res.computed) return std::nullopt;

  // Лидар не различает разворот на 180° — подгоняем под текущий гироскоп.
  double angle1 = gyro_angle - top_angle;
  double angle2 = res.rotation;
  if (abs(normalize_angle(angle1 - angle2)) > M_PI / 2) {
    res.v = res.v * -1;
    res.rotation += M_PI;
  }

  LidarPose measured{field_dims::kCenter + res.v,
                     normalize_angle(res.rotation)};

  if (!config->lidar->calibration->enabled) return measured;

  long long now = millis();
  lidar_history.push_back({now, gyro_angle, position});

  long long window_ms = config->lidar->calibration->window;
  long long threshold_ms = config->lidar->calibration->threshold;

  while (!lidar_history.empty() &&
         now - lidar_history.front().time > window_ms + threshold_ms) {
    lidar_history.pop_front();
  }

  if (lidar_history.size() >= 2 &&
      now - lidar_history.front().time >= window_ms) {
    double total_rotation = 0.0;
    double total_movement = 0.0;
    for (size_t i = 1; i < lidar_history.size(); ++i) {
      total_rotation += abs(normalize_angle(
          lidar_history[i].gyro_angle - lidar_history[i - 1].gyro_angle));
      total_movement +=
          (lidar_history[i].position - lidar_history[i - 1].position).len();
    }
    if (total_rotation < config->lidar->calibration->angle &&
        total_movement < config->lidar->calibration->movement) {
      top_angle = normalize_angle(gyro_angle - measured.field_angle);
      lidar_history.clear();
      spdlog::info(
          "Lidar drift calibration: rotation={:.3f} movement={:.2f}",
          total_rotation, total_movement);
    }
  }

  return measured;
}

void Robot::compute_gyro_angle() {
  field_angle = normalize_angle(gyro_angle - top_angle);
}

void Robot::predict_position(double dt) {
  position = position + actual_vel * dt;
}

void Robot::apply_motion_limits(double dt) {
  if (dt <= 0) return;
  double max_linear_step = config->strategy->motion->max_linear_accel * dt;
  Vec delta = vel - actual_vel;
  if (delta.len() > max_linear_step) {
    delta = delta.resize(max_linear_step);
  }
  actual_vel += delta;
}

double Robot::relative_angle(const Vec& p) const {
  return (p - position).field_angle() - field_angle;
}

double Robot::resolve_rotation() const {
  const auto& st = config->strategy->safe_turn;
  if (position.y < st->y &&
      std::abs(normalize_angle(rotation)) > st->angle) {
    // Крутимся через направление к ближайшей боковой стенке.
    double wall_angle =
        position.x < field_dims::kWidth / 2 ? M_PI / 2 : -M_PI / 2;
    double delta = normalize_angle(wall_angle - field_angle);
    return (delta >= 0 ? 1.0 : -1.0) * M_PI / 2;
  }
  return rotation;
}

Vec Robot::ball_hole_position() const {
  Vec d = Vec{field_angle}.resize(9.5);
  return position + d;
}