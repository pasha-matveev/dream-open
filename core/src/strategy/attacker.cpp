#include "strategy/strategy.h"

void Strategy::run_attacker(Robot &robot, Ball &ball) {
  robot.compute_lidar();

  if (robot.emitter) {
    Vec target{91, 243};
    Vec v = target - robot.position;
    Vec robot_dir{-1 * sin(robot.field_angle), cos(robot.field_angle)};
    double ang = atan2(robot_dir % v, robot_dir * v);
    robot.rotation_limit = 10;
    robot.dribling = 50;
    robot.rotation = ang;
    robot.speed = 0;
    robot.kicker_force = 0;
    if (abs(ang) <= 0.05) {
      robot.kicker_force = 30;
    }
  } else if (ball.visible) {
    robot.dribling = 40;
    robot.rotation = ball.relative_angle;
    robot.rotation_limit = 30;
    if (ball.get_cm() > 10) {
      robot.speed = 40;
    } else {
      robot.speed = 10;
    }
    robot.direction = ball.relative_angle;
    robot.kicker_force = 0;
  } else {
    robot.rotation = 0;
    robot.speed = 0;
    robot.kicker_force = 0;
  }
}