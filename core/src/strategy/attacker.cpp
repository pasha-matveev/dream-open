#include "strategy/strategy.h"
#include "utils/millis.h"

void Strategy::run_attacker(Robot& robot, Object& ball, Object& goal) {
  const int BORDER = 93;
  bool straight_path = true;
  bool reset_target = true;

  if (millis() - last_ball_visible < 3000) {
    if (!robot.emitter) {
      Vec target = last_ball;
      if (last_ball.y < BORDER) {
        target = {160, 120};
      }
      robot.vel = target - robot.position;
      robot.vel *= 4;
      robot.rotation =
          (target - robot.position).field_angle() - robot.field_angle;
      robot.rotation_limit = 30;
      robot.dribling = 60;
    } else {
      int power;
      if (robot.position.y < 146) {
        power = 50;
      } else if (robot.position.y < 180) {
        power = 30;
      } else {
        power = 10;
      }
      hit(robot, goal, true, power);
    }
  } else {
    Vec target{91, 121};
    robot.vel = target - robot.position;
    robot.vel = robot.vel.resize(min(robot.vel.len(), 40.0));
    robot.rotation = 0;
    robot.rotation_limit = 30;
  }
  robot.vel = robot.vel.resize(min(robot.vel.len(), 80.0));
}