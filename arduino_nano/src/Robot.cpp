#include "Robot.h"

#include <Adafruit_NeoPixel.h>

void Robot::init(Adafruit_NeoPixel& pixels) {
  // Зеленый пока все обычные компоненты
  pixels.setPixelColor(0, pixels.Color(0, 50, 0));
  pixels.show();

  if (init_motors) motors.init();
  if (init_dribling) dribling.init();
  if (init_emitter) emitter.init();
  if (init_button) button.init();
  if (init_kicker) kicker.init();

  // Белый пока гироскоп
  pixels.setPixelColor(0, pixels.Color(50, 50, 50));
  pixels.show();

  if (init_gyro) gyro.init();

  // Выключить
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
}

void Robot::read() {
  if (init_gyro) gyro.read();
  if (init_emitter) emitter.read();
}

void Robot::run() {
  rel_direction = gyro.relative(direction);
  rel_rotation = gyro.relative(rotation);
  motors.run(rel_direction, speed, rel_rotation, rotation_limit);
  if (init_dribling) dribling.run();
  first_motors_stop = true;
}

void Robot::stop() {
  kicker_force = 0;
  if (first_motors_stop) {
    motors.run(0, 0, 0);
    first_motors_stop = false;
  } else {
    motors.stop();
  }
  if (init_dribling) {
    // dribling.set_speed(0);
    dribling.run();
  }
}

void Robot::reset() {
  speed = 0;
  rotation_limit = 0;
  dribling_speed = 0;
  kicker_force = 0;
}