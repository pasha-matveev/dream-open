#pragma once

#include "gpio/buzzer.h"
#include "gpio/display.h"
#include "tracking/ball.h"
#include "tracking/camera.h"
#include "utils/lidar.h"
#include "utils/uart.h"
#include "utils/vec.h"

using namespace LibSerial;

class Robot {
 private:
  void init_camera(Ball&);
  void init_buzzer();
  void init_buttons();
  void init_uart();
  void init_gyro();
  void init_display();
  void init_lidar();

  double top_angle;

  float direction = 0;
  float speed = 0;

 public:
  Camera* camera = nullptr;
  Buzzer* buzzer = nullptr;
  UART* uart = nullptr;
  Display* display = nullptr;
  Lidar* lidar = nullptr;

  Robot() = default;
  ~Robot();

  // Координаты центра робота на поле, в сантиметрах.
  // Оси координат направлены математически
  // Наши ворота снизу, вражеские - сверху
  Vec position = {182 / 2, 243 / 2};
  // Фактическое положение.
  // 0 - на ворота противника
  // P/2 - налево
  // -P/2 - направо
  float field_angle = M_PI / 2;

  float gyro_angle = 0;
  bool emitter = false;
  bool prev_emitter = false;
  int first_time = 0;
  bool kicker_charged = false;

  Vec vel{0, 0};
  float rotation = 0;
  float rotation_limit = 100;
  int dribling = 0;
  int kicker_force = 0;
  bool rgb_led = false;

  void init_hardware(Ball& ball);

  bool compute_lidar();
  void compute_gyro_angle();

  void read_from_arduino();
  void write_to_arduino();

  void predict_position();
};