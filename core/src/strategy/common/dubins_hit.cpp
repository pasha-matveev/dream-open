#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/geo/circle.h"

void Strategy::dubins_hit(Robot& robot, Object& goal, int power, bool control) {
  robot.dribling = config.strategy.base_dribling;

  Vec goal_direction;

  if (goal.visible && (last_ball_position - robot.position).len() <= 5) {
    // Подъехали близко к мячу, можно целиться по камере
    double dir_angle = goal.relative_angle + robot.field_angle;
    goal_direction = Vec{dir_angle};
  } else {
    // Целимся по логике
    Vec target{91, 240};
    goal_direction = target - last_ball_position;
  }

  Vec destination =
      last_ball_position + goal_direction.resize(-config.strategy.dubins.bonus);
  Vec left_center =
      destination +
      goal_direction.turn_left().resize(config.strategy.dubins.radius) +
      goal_direction.turn_right().resize(config.strategy.dubins.separate);
  Vec right_center =
      destination +
      goal_direction.turn_right().resize(config.strategy.dubins.radius) +
      goal_direction.turn_left().resize(config.strategy.dubins.separate);
  Circle left_circle{left_center, config.strategy.dubins.radius};
  Circle right_circle{right_center, config.strategy.dubins.radius};

  bool left;
  if (abs(left_circle.dist(robot.position)) <
      abs(right_circle.dist(robot.position))) {
    left = true;
  } else {
    left = false;
  }

  Circle& circle = left ? left_circle : right_circle;

  circle.draw();

  if (robot.emitter) {
    if (control) {
      kick_to_goal(robot, goal, {});
    } else {
      robot.kicker_force = power;
    }
    return;
  }

  double kick_angle =
      normalize_angle(goal_direction.field_angle() -
                      (last_ball_position - robot.position).field_angle());

  Vec movement_direction;
  double len;
  if (abs(kick_angle) <= 0.04) {
    // Едем напрямую к мячу
    Vec vel = last_ball_position - robot.position;
    movement_direction = vel;
    len = vel.len();
  } else if (circle.inside(robot.position) ||
             circle.dist(robot.position) <= 0) {
    // Внутри круга / на круге
    Vec center_dir = circle.center - robot.position;
    if (left) {
      movement_direction = center_dir.turn_right();
    } else {
      movement_direction = center_dir.turn_left();
    }
    len = circle.path_len(robot.position, destination, left) +
          config.strategy.dubins.bonus;
  } else {
    // Едем к кругу по касательной
    Vec tangent;
    if (left) {
      tangent = left_circle.tangent_right(robot.position);
    } else {
      tangent = right_circle.tangent_left(robot.position);
    }
    movement_direction = tangent - robot.position;
    len = circle.path_len(tangent, destination, left) +
          config.strategy.dubins.bonus + movement_direction.len();
  }
  double speed = config.strategy.dubins.speed.map(len);
  robot.vel = movement_direction.resize(speed);

  double target_relative =
      normalize_angle(goal_direction.field_angle() - robot.field_angle);

  robot.rotation = target_relative;
}