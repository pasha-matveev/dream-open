#include "strategy/strategy.h"
#include "utils/config.h"
#include "utils/geo/circle.h"
#include "utils/nmap.h"

void Strategy::dubins_hit(Robot& robot, Object& goal) {
  if (robot.emitter) {
    robot.kicker_force = 30;
    return;
  }
  reset_dubins = false;
  double dist = (last_ball - robot.position).len();
  // TODO: dynamic target
  Vec target{91, 240};
  Vec dir = target - last_ball;
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
  if (dubins_side == DubinsSide::NONE) {
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

  double angle = normalize_angle(dir.field_angle() - last_ball_relative -
                                 robot.field_angle);

  double speed = nmap(
      dist, config.strategy.dubins.base_dist, config.strategy.dubins.max_dist,
      config.strategy.dubins.base_speed, config.strategy.dubins.max_speed);
  Vec vel;
  if (abs(angle) <= 0.01) {
    // Едем напрямую к мячу
    vel = last_ball - robot.position;
  } else if (circle.inside(robot.position) ||
             circle.dist(robot.position) <= 0) {
    // Внутри круга / на круге
    Vec dir = circle.center - robot.position;
    if (left) {
      dir = dir.turn_right();
    } else {
      dir = dir.turn_left();
    }
    // TODO: speed
    vel = dir.resize(40);
  } else {
    // Едем к кругу по касательной
    Vec t;
    if (left) {
      t = left_circle.tangent_right(robot.position);
    } else {
      t = right_circle.tangent_left(robot.position);
    }
    // cout << t.x << " " << t.y << endl;
    vel = (t - robot.position);
  }
  vel = vel.resize(speed);
  robot.vel = vel;
  robot.rotation = last_ball_relative;
}