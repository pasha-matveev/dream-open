#include "robot.h"

#include <libserial/SerialPort.h>
#include <spdlog/spdlog.h>

#include <iostream>
#include <thread>

#include "gpio/buttons.h"
#include "gpio/setup.h"
#include "media/img.h"
#include "utils/config.h"

void Robot::read_from_arduino() {
  uart->write_data<char>('R');
  gyro_angle = normalize_angle2(-1 * uart->read_data<float>());
  emitter = uart->read_data<bool>();
  kicker_charged = uart->read_data<bool>();
}

void Robot::write_to_arduino() {
  speed = vel.len();
  direction = vel.field_angle() - field_angle;
  uart->write_data<char>('W');
  uart->write_data<float>(
      normalize_angle2(-gyro_angle - normalize_angle2(direction)));
  uart->write_data<float>(speed);
  uart->write_data<float>(
      normalize_angle2(-gyro_angle - normalize_angle2(rotation)));
  uart->write_data<float>(rotation_limit);
  uart->write_data<int32_t>(dribling);
  uart->write_data<int32_t>(kicker_force);
  uart->write_data<bool>(rgb_led);
}

void Robot::init_camera(Ball& ball) {
  camera = new Camera(ball);
  camera->start();
}

void Robot::init_buzzer() { buzzer = new Buzzer(config.gpio.buzzer.pin); }

void Robot::init_buttons() { setup_buttons(buzzer); }

void Robot::init_uart() {
  uart = new UART();
  uart->connect();
  uart->wait_for_x();
}

void Robot::init_display() {
  string str_address = config.gpio.display.address;
  uint8_t address = stoi(str_address, nullptr, 16);
  display = new Display(config.gpio.display.device.c_str(), address);
  display->init();
  if (config.gpio.display.mode == "img") {
    display->draw_image(get_img(config.gpio.display.img));
  }
}

void Robot::init_lidar() {
  lidar = new Lidar();
  lidar->start();
  while (!lidar->received_data) {
    this_thread::sleep_for(chrono::milliseconds(50));
  }
}

void Robot::init_hardware(Ball& ball) {
  if (config.tracking.enabled) {
    init_camera(ball);
  }
  spdlog::info("Camera ready");
  if (config.gpio.enabled) {
    setup_wiringpi();
    if (config.gpio.buzzer.enabled) {
      init_buzzer();
    }
    spdlog::info("Buzzer ready");
    if (config.gpio.buttons.enabled) {
      init_buttons();
    }
    spdlog::info("Buttons ready");
    if (config.gpio.display.enabled) {
      init_display();
    }
    spdlog::info("Camera ready");
  }
  spdlog::info("GPIO ready");
  if (config.serial.enabled) {
    init_uart();
    spdlog::info("Reading from arduino");
    read_from_arduino();
    spdlog::info("Got data from arduino");
    init_gyro();
  }
  spdlog::info("UART ready");
  if (config.lidar.enabled) {
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

void Robot::init_gyro() { top_angle = normalize_angle2(gyro_angle + M_PI / 2); }

bool Robot::compute_lidar() {
  if (!lidar) return false;
  auto res = lidar->compute();

  if (!res.computed) return false;

  double angle1 = gyro_angle - top_angle;
  double angle2 = res.rotation;
  bool inv = false;
  if (abs(normalize_angle(angle1 - angle2)) > M_PI / 2) {
    inv = true;
    res.v = res.v * -1;
    res.rotation += M_PI;
  }
  // cout << "Raw gyro: " << normalize_angle(gyro_angle) << '\n';
  // cout << "Computed gyro: " << normalize_angle(angle1) << '\n';
  // cout << "Lidar: " << normalize_angle(angle2) << '\n';
  // cout << "Inversion: " << inv << endl;

  Vec center = {182.0 / 2, 243.0 / 2};
  position = center + res.v;
  // cout << "Position: " << position.x << " " << position.y << endl;
  field_angle = normalize_angle(res.rotation);

  top_angle = normalize_angle(gyro_angle - field_angle);

  return true;
}

void Robot::compute_gyro_angle() {
  field_angle = normalize_angle(gyro_angle - top_angle);
}

void Robot::predict_position() {
  // double len = speed / 60;
  // double direction = field_angle + direction;
  // Vec shift = {-1 * sin(direction) * len, cos(direction) * len};
  // position = position + shift;

    // field_angle +=
    //     clamp(rotation / 60.0, -1.0 * rotation_limit / 60.0, rotation_limit / 60.0);
}
