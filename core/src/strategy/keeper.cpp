#include <algorithm>
#include <cmath>

#include "strategy/strategy.h"

void Strategy::run_keeper(Robot& robot, Ball& ball) {
  if (!ball.visible) {
    robot.speed = 0;
    robot.rotation = 0;
    while (!st.empty()) {
      st.pop();
    }
    return;
  }

  double ball_angle = robot.field_angle + ball.relative_angle;
  Vec ball_position = Vec{ball.get_cm() * sin(ball_angle) * -1,
                          ball.get_cm() * cos(ball_angle)} +
                      robot.position;
  optional<Vec> last_position;
  st.push(ball_position);
  if (st.size() >= 3) {
    last_position = st.top();
    st.pop();
  }

  bool line_movement = false;

  Vec target_position = {ball_position.x, 55.0};
  if (last_position.has_value()) {
    double delta_y = last_position->y - ball_position.y;
    double target_delta_y = last_position->y - 55;
    if (delta_y > 1 && target_delta_y > 0) {
      line_movement = true;
      double delta_x = ball_position.x - last_position->x;
      double target_delta_x = delta_x / delta_y * target_delta_y;
      double target_x = last_position->x + target_delta_x;
      target_position = {target_x, 55.0};
    }
  }
  target_position.x = std::clamp(target_position.x, 40.0, 130.0);
  Vec vel = target_position - robot.position;

  if (ball_position.y < 85 || line_movement) {
    robot.speed = abs(vel.x) * 20;
  } else {
    robot.speed = abs(vel.x) * 10;
  }

  if (ball_position.y < 50) {
    robot.speed = 0;
  }
  // robot.speed = min(robot.speed, 30.0f);
  robot.direction = vel.field_angle() - robot.field_angle;
  robot.rotation = ball.relative_angle;
  robot.rotation_limit = 10;
}
