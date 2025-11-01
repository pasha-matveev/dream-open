#include "Dribling.h"

#include <Arduino.h>

#include "Robot.h"

void Dribling::init() {
  ESC1.attach(ESC1_pin);
  ESC2.attach(ESC2_pin);
  delay(1000);
  ESC1.writeMicroseconds(800);
  ESC2.writeMicroseconds(800);
  delay(2000);
  ESC1.writeMicroseconds(2200);
  ESC2.writeMicroseconds(2200);
  delay(2000);
  ESC1.writeMicroseconds(1450);
  ESC2.writeMicroseconds(1450);
  delay(3000);

  set_speed(0);
}

void Dribling::set_speed(int rotation) { desired_speed = (float)rotation; }

void Dribling::run() {
  if (lst_tm + 100 < millis()) {
    if (current_speed < desired_speed)
      current_speed += 5;
    else if (current_speed > desired_speed)
      current_speed -= 5;
    lst_tm = millis();
  }

  float sp1, sp2;
  sp1 = current_speed ? map(current_speed, 0, 100, 1440, 1600) : 1450;
  sp2 = current_speed ? map(current_speed, 0, 100, 1440, 1600) : 1450;

  ESC1.writeMicroseconds(sp1);
  ESC2.writeMicroseconds(sp2);
}