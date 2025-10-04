#pragma once

#include "gpio/buzzer.h"
#include "gpio/display.h"
#include "tracking/camera.h"
#include "utils/lidar.h"
#include "utils/uart.h"

using namespace LibSerial;

class Robot {
 private:
  void init_camera();
  void init_buzzer();
  void init_buttons();
  void init_uart();
  void init_display();
  void init_lidar();

 public:
  Camera *camera = nullptr;
  Buzzer *buzzer = nullptr;
  UART *uart = nullptr;
  Display *display = nullptr;
  Lidar *lidar = nullptr;

  Robot() = default;
  ~Robot();

  float angle = 0;
  bool emitter = false;
  bool kicker_charged = false;

  float direction = 0;
  float speed = 0;
  float rotation = 0;
  float rotation_limit = 30;
  int dribling = 0;
  int kicker_force = 0;
  bool rgb_led = false;

  void init_hardware();

  void read_from_arduino();
  void write_to_arduino();
};