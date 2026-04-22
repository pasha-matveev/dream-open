#include "Dribling.h"

#include <Arduino.h>

#include "Robot.h"

void Dribling::init() {
  ESC_left.attach(left_pin);
  if (ROBOT) ESC_right.attach(right_pin);
  delay(1000);
  ESC_left.writeMicroseconds(800);
  if (ROBOT) ESC_right.writeMicroseconds(800);
  delay(2000);
  ESC_left.writeMicroseconds(2200);
  if (ROBOT) ESC_right.writeMicroseconds(2200);
  delay(2000);
  ESC_left.writeMicroseconds(1450);
  if (ROBOT) ESC_right.writeMicroseconds(1450);
  delay(3000);
}

// void Dribling::set_speed(int rotation) { desired_speed = (float)rotation; }
void Dribling::set_rmp(int value) { desired_rpm = value; }

void Dribling::run() {
  // double k = 20.0 / 100;
  // int delta = millis() - lst_tm;
  // if (delta >= 10) {
  //   int diff = abs(current_speed - desired_speed);
  //   if (current_speed < desired_speed) {
  //     current_speed += min(delta * k, diff);
  //   } else if (current_speed > desired_speed) {
  //     current_speed -= min(delta * k, diff);
  //   }

  //   lst_tm = millis();
  // }

  // float sp1, sp2;

  // int speed1 =

  int raw_left = left_k * desired_rpm + left_b;
  int raw_right = right_k * (desired_rpm + 500) + right_b;

  int power_left = constrain(raw_left, 1450, 1600);
  int power_right = constrain(raw_right, 1450, 1600);

  ESC_left.writeMicroseconds(power_left);

  if (ROBOT) {
    // Ignore right motor on black robot
    ESC_right.writeMicroseconds(power_right);
  }
}