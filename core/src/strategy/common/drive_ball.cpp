#include "strategy/strategy.h"
#include "utils/config.h"

void Strategy::drive_ball(Robot& robot, const Vec& ball) {
  double dist = (ball - robot.position).len();
  Vec dir = ball - robot.position;
  double dist = dir.len();
  double speed = config.strategy.drive.ball_speed.map(dist);
  Vec vel = dir.resize(speed);
  robot.dribling = config.strategy.dribbling.value_l;
  robot.rotation_limit = 40;
  robot.vel = vel;
}