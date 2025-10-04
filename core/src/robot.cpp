#include "robot.h"

#include <libserial/SerialPort.h>

#include <iostream>
#include <thread>

#include "gpio/buttons.h"
#include "gpio/setup.h"
#include "media/img.h"
#include "spdlog/spdlog.h"
#include "utils/config.h"

void Robot::read_from_arduino() {
  uart->write_data<char>('R');
  gyro_angle = uart->read_data<float>();
  emitter = uart->read_data<bool>();
  kicker_charged = uart->read_data<bool>();
}

void Robot::write_to_arduino() {
  uart->write_data<char>('W');
  uart->write_data<float>(direction);
  uart->write_data<float>(speed);
  uart->write_data<float>(rotation);
  uart->write_data<float>(rotation_limit);
  uart->write_data<int32_t>(dribling);
  uart->write_data<int32_t>(kicker_force);
  uart->write_data<bool>(rgb_led);
}

void Robot::init_camera(Ball &ball) {
  camera = new Camera(ball);
  camera->start();
}

void Robot::init_buzzer() {
  buzzer = new Buzzer(config["gpio"]["buzzer"]["pin"].GetInt());
}

void Robot::init_buttons() { setup_buttons(buzzer); }

void Robot::init_uart() {
  uart = new UART();
  uart->connect();
  uart->wait_for_x();
}

void Robot::init_display() {
  string str_address = config["gpio"]["display"]["address"].GetString();
  uint8_t address = stoi(str_address, nullptr, 16);
  display =
      new Display(config["gpio"]["display"]["device"].GetString(), address);
  display->init();
  if (string(config["gpio"]["display"]["mode"].GetString()) == "img") {
    display->draw_image(get_img(config["gpio"]["display"]["img"].GetInt()));
  }
}

void Robot::init_lidar() {
  lidar = new Lidar();
  lidar->start();
}

void Robot::init_hardware(Ball &ball) {
  if (config["tracking"]["enabled"].GetBool()) {
    init_camera(ball);
  }
  spdlog::info("Camera ready");
  if (config["gpio"]["enabled"].GetBool()) {
    setup_wiringpi();
    if (config["gpio"]["buzzer"]["enabled"].GetBool()) {
      init_buzzer();
    }
    spdlog::info("Buzzer ready");
    if (config["gpio"]["buttons"]["enabled"].GetBool()) {
      init_buttons();
    }
    spdlog::info("Buttons ready");
    if (config["gpio"]["display"]["enabled"].GetBool()) {
      init_display();
    }
    spdlog::info("Camera ready");
  }
  spdlog::info("GPIO ready");
  if (config["serial"]["enabled"].GetBool()) {
    init_uart();
  }
  spdlog::info("UART ready");
  if (config["lidar"]["enabled"].GetBool()) {
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

Vec Robot::ball_position(Ball &ball) {
  if (!ball.visible) {
    spdlog::error("Tried to get ball position while it is not visible");
    return {-1, -1};
  }
  return position + ball.field_point;
}