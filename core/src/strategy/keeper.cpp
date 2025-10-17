#include "strategy/strategy.h"

Vec last_ball_position{-1, -1};

void Strategy::run_keeper(Robot& robot, Ball& ball) {
  if (!ball.visible) {
    robot.speed = 0;
    robot.rotation = 0;
    return;
  }
  double ball_angle = robot.field_angle + ball.relative_angle;
  Vec ball_position = Vec{ball.get_cm() * sin(ball_angle) * -1,
                          ball.get_cm() * cos(ball_angle)} +
                      robot.position;
  Vec target_position = {ball_position.x, 55.0};
  target_position.x = min(target_position.x, 130.0);
  target_position.x = max(target_position.x, 30.0);
  Vec vel = target_position - robot.position;

  if (ball_position.y < 85) {
    robot.speed = abs(vel.x) * 20;
  } else {
    robot.speed = abs(vel.x) * 10;
  }

  if (ball_position.y < 50) {
    robot.speed = 0;
  }
  robot.direction = vel.field_angle() - robot.field_angle;
  robot.rotation = ball.relative_angle;
  robot.rotation_limit = 10;

  last_ball_position = ball_position;
}