#include "strategy/strategy.h"
#include "utils/millis.h"

constexpr int UP_TIME = 300;

bool Strategy::turn(Robot& robot, double target_angle, bool curved_rotation,
                    bool ac_dribling) {
  reset_turn = false;
  if (turn_time == -1) {
    turn_time = millis();
  }
  target_angle -= 0.01;  // Бьём немного правее
  double delta = target_angle - robot.field_angle;
  long long passed = millis() - turn_time;
  double k = min(1.0, (double)passed / UP_TIME);
  if (curved_rotation) {
    if (abs(delta) <= 0.035) {
      robot.vel = {0, 0};
      robot.rotation_limit = 15.0;
    } else {
      Vec vel{robot.field_angle};
      if (delta > 0) {
        vel = vel.turn_right();
        vel = vel.rotate(0.1);
      } else {
        vel = vel.turn_left();
        vel = vel.rotate(-0.1);
      }
      // Длина (17) и скорость поворота (15) подобраны
      vel = vel.resize(19.0 * k);
      robot.vel = vel;
      robot.rotation_limit = 15.0 * k;
    }
  } else {
    robot.vel = {0, 0};
    robot.rotation_limit = 15.0 * k;
  }
  robot.rotation = delta;
  desired_dribling(robot, ac_dribling);
  if (abs(delta) <= 0.015) {
    // поворот закончен
    return true;
  }
  return false;
}