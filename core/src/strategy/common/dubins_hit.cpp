#include <spdlog/spdlog.h>

#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/geo/circle.h"

bool circle_ok(Circle& circle, Field& field) {
  if (!field.inside(circle.center)) return false;
  double dist = field.dist(circle.center) - circle.r;
  // чтобы робот проехал, должна быть хотя бы половина корпуса
  return dist >= 10;
}

bool Strategy::dubins_hit(Robot& robot, Object& goal, Field& field, int _,
                          bool control) {
  if (robot.emitter) {
    cur_dubins = true;
    if (control) {
      kick_to_goal(robot, goal, {});
    } else {
      robot.kicker_force = compute_power(robot.position.y);
    }
    return true;
  }

  Vec goal_direction;

  if (goal.visible && (last_ball_position - robot.position).len() <=
                          config.strategy.dubins.camera_target_dist) {
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
  Vec left_center = destination + goal_direction.turn_left().resize(
                                      config.strategy.dubins.radius +
                                      config.strategy.dubins.separate);
  Vec right_center = destination + goal_direction.turn_right().resize(
                                       config.strategy.dubins.radius +
                                       config.strategy.dubins.separate);

  Circle left_circle{left_center, config.strategy.dubins.radius};
  Circle right_circle{right_center, config.strategy.dubins.radius};
  bool left_ok = circle_ok(left_circle, field);
  bool right_ok = circle_ok(right_circle, field);

  if (!left_ok && !right_ok) {
    return false;
  }
  cur_dubins = true;

  bool left;
  if (left_ok && right_ok) {
    if (abs(left_circle.dist(robot.position)) <
        abs(right_circle.dist(robot.position))) {
      left = true;
    } else {
      left = false;
    }
  } else if (left_ok) {
    left = true;
  } else {
    left = false;
  }

  Circle& circle = left ? left_circle : right_circle;
  Circle& inactive_circle = left ? right_circle : left_circle;

  circle.draw(true);
  inactive_circle.draw(false);

  // Метрика "на линии удара" в сантиметрах: lateral — поперечное смещение
  // от прямой ball->goal, longitudinal — продольное (отрицательное = робот
  // сзади мяча со стороны, противоположной воротам). Так шум позиции не
  // раскачивает порог при приближении к мячу.
  Vec shot_dir = goal_direction.resize(1);
  Vec ball_to_robot = robot.position - last_ball_position;
  double longitudinal = ball_to_robot * shot_dir;
  double lateral = abs(ball_to_robot % shot_dir);
  bool on_shot_line = config.strategy.dubins.kick_precision.compute(lateral);

  Vec movement_direction;
  double len;
  robot.dribling = config.strategy.dribbling_slow;
  if (on_shot_line && longitudinal < 0) {
    // На линии удара и сзади мяча. Целимся в точку за мячом со стороны
    // ворот — движение автоматически стягивает остаточный поперечный промах.
    Vec aim =
        last_ball_position + shot_dir * config.strategy.dubins.bonus;
    movement_direction = aim - robot.position;
    len = (last_ball_position - robot.position).len();
  } else if (circle.dist(robot.position) < 0 &&
             abs(circle.dist(robot.position)) >
                 config.strategy.dubins.deep_inside) {
    // Глубоко внутри круга // Выталкиваемся в точку напротив ворот
    Vec t = goal_direction.resize(-circle.r) + circle.center;
    movement_direction = t - robot.position;
    len = 0;
    len += movement_direction.len();
    len += circle.path_len(t, destination, left);
    len += config.strategy.dubins.bonus;
  } else if (circle.inside(robot.position)) {
    // На круге
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

  robot.rotation = last_ball_relative_angle(robot);

  return true;
}