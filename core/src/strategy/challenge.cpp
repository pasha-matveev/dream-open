#include "strategy/strategy.h"
#include "utils/millis.h"

void Strategy::run_challenge(Robot& robot, Object& ball, Object& goal) {
  if (last_ball_visible >= millis() - 1000) {
    Vec target = last_ball;
    robot.rotation = normalize_angle((target - robot.position).field_angle() -
                                     robot.field_angle);
    Vec vel = ball.field_position - robot.position;
    vel *= 2.5;
    robot.vel = vel;
  }
}