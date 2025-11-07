#include "strategy/strategy.h"
#include "utils/millis.h"

void Strategy::run_attacker(Robot& robot, Object& ball, Object& goal) {
  const int BORDER = 80;

  if (robot.emitter) {
    int power = 70;
    if (robot.position.y < 146) {
      power = 40;
    } else if (robot.position.y < 180) {
      power = 20;
    } else {
      power = 15;
    }
    hit(robot, goal, power = power);
  } else {
    if (millis() - last_ball_visible < 3000) {
      if (last_ball.y < BORDER) {
        drive_target(robot, {160, 120}, 4);
      } else {
        drive_ball(robot, last_ball);
      }
    } else {
      drive_target(robot, {91, 121}, 3);
    }
  }

  robot.vel = robot.vel.resize(min(robot.vel.len(), 120.0));
}