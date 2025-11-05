#include "strategy/strategy.h"
#include "utils/millis.h"

void Strategy::run_attacker(Robot& robot, Object& ball, Object& goal) {
  const int BORDER = 0;
  bool straight_path = true;
  bool reset_target = true;

  if (millis() - last_ball_visible < 3000) {
    if (!robot.emitter) {
      Vec target = last_ball;
      if (last_ball.y < BORDER) {
        target = {160, 120};
      }
      robot.vel = target - robot.position;
      if (robot.vel.len() > 45) {
        robot.vel *= 3;
      } else {
        robot.vel *= 4;
      }
      robot.rotation =
          (target - robot.position).field_angle() - robot.field_angle;
      robot.rotation_limit = 30;
      robot.dribling = 50;
    } else {
      int power = 70;
      if (robot.position.y < 146) {
        power = 40;
      } else if (robot.position.y < 180) {
        power = 20;
      } else {
        power = 15;
      }
      hit(robot, goal, false, power, true);
    }
  } else {
    Vec target{91, 121};
    robot.vel = target - robot.position;
    robot.vel = robot.vel.resize(min(robot.vel.len(), 40.0));
    robot.rotation = 0;
    robot.rotation_limit = 30;
  }
  robot.vel = robot.vel.resize(min(robot.vel.len(), 120.0));
}