#include "strategy/strategy.h"

void Strategy::run_keeper(Robot &robot, Ball &ball) {
  if (!ball.visible) {
    robot.speed = 0;
    robot.rotation = 0;
    return;
  }
  double ball_angle = robot.field_angle + ball.relative_angle;
  Vec ball_position = Vec{ball.get_cm() * sin(ball_angle) * -1,
                          ball.get_cm() * cos(ball_angle)} +
                      robot.position;
  Vec target_position = {ball_position.x, 30.0};
  cout << target_position.x << " " << target_position.y << endl;
  double desired_speed = (target_position - robot.position).len();

  if (desired_speed < 1) {
    robot.speed = desired_speed;
  } else {
    robot.speed = max(desired_speed, 50.0);
  }

  robot.direction =
      (target_position - robot.position).field_angle() - robot.field_angle;
  robot.rotation = ball.relative_angle;
  robot.rotation_limit = 30;
}