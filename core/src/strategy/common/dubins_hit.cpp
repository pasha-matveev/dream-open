#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/geo/circle.h"
#include "utils/nmap.h"

void Strategy::dubins_hit(Robot& robot, Object& goal, int power, bool control) {
  robot.dribling = config.strategy.base_dribling;

  reset_dubins = false;
  double dist = (last_ball - robot.position).len();
  Vec dir;
  if (goal.camera_visible) {
    double dir_angle = goal.relative_angle + robot.field_angle;
    dir = Vec{dir_angle};
  } else {
    Vec target{91, 240};
    // dir = target - last_ball;
    dir = target - robot.position;
  }

  Vec a = last_ball + dir.resize(-config.strategy.dubins.bonus);
  Vec left_center = a + dir.turn_left().resize(config.strategy.dubins.radius) +
                    dir.turn_right().resize(config.strategy.dubins.delta);
  Vec right_center = a +
                     dir.turn_right().resize(config.strategy.dubins.radius) +
                     dir.turn_left().resize(config.strategy.dubins.delta);
  Circle left_circle{left_center, config.strategy.dubins.radius};
  Circle right_circle{right_center, config.strategy.dubins.radius};

  if ((last_ball - dubins_ball).len() > config.strategy.dubins.threshold) {
    dubins_side = DubinsSide::NONE;
  }
  if (true) {
    dubins_ball = last_ball;
    if (abs(left_circle.dist(robot.position)) <
        abs(right_circle.dist(robot.position))) {
      dubins_side = DubinsSide::LEFT;
    } else {
      dubins_side = DubinsSide::RIGHT;
    }
  }
  bool left = dubins_side == DubinsSide::LEFT;

  Circle& circle = left ? left_circle : right_circle;

  circle.draw();

  double angle = normalize_angle(dir.field_angle() - last_ball_relative -
                                 robot.field_angle);

  double speed = nmap(
      dist, config.strategy.dubins.base_dist, config.strategy.dubins.max_dist,
      config.strategy.dubins.base_speed, config.strategy.dubins.max_speed);
  Vec vel;
  if (robot.emitter) {
    if (control) {
      cout << "CONTROL" << endl;
      hit(robot, goal, power, 400, true, 0, 0.03, false);
      reset_dubins = true;
      return;
    } else if (abs(angle) <= 0.5) {
      cout << "NO CONTROL" << endl;
      robot.kicker_force = power;
      return;
    }
  }
  if (abs(angle) <= 0.04) {
    // Едем напрямую к мячу
    vel = last_ball - robot.position;
  } else if (circle.inside(robot.position) ||
             circle.dist(robot.position) <= 0) {
    // Внутри круга / на круге
    // if (abs(circle.dist(robot.position)) >=
    //     config.strategy.dubins.inside_threshold) {
    //   // Мы глубоко внутри круга
    // }
    Vec center_dir = circle.center - robot.position;
    if (left) {
      center_dir = center_dir.turn_right();
    } else {
      center_dir = center_dir.turn_left();
    }
    vel = center_dir.resize(speed);
  } else {
    // Едем к кругу по касательной
    Vec t;
    if (left) {
      t = left_circle.tangent_right(robot.position);
    } else {
      t = right_circle.tangent_left(robot.position);
    }
    vel = (t - robot.position);
  }
  vel = vel.resize(speed);
  robot.vel = vel;
  double target_relative =
      normalize_angle(dir.field_angle() - robot.field_angle);
  // robot.rotation =
  // normalize_angle(last_ball_relative / 2 + target_relative / 2);
  robot.rotation = target_relative;
  robot.rotation_limit = 30;
}