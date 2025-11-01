#include "strategy/strategy.h"

void Strategy::run_attacker(Robot& robot, Object& ball, Object& goal) {
  const int BORDER = 80;
  bool straight_path = true;
  bool reset_target = true;

  if (millis() - last_ball_visible < 3000) {
    if (!robot.emitter) {
      Vec target = last_ball;
      if (last_ball.y < BORDER) {
        target = {20, 120};
      }
      robot.vel = last_ball - robot.position;
      // robot.vel *= 3;
      robot.rotation =
          (last_ball - robot.position).field_angle() - robot.field_angle;
      robot.rotation_limit = 30;
      robot.dribling = 60;
    } else {
      if (millis() - robot.first_time < 500) {
        robot.vel = last_ball - robot.position;
        robot.vel = robot.vel.resize(10);
        robot.dribling = 60;
        robot.rotation_limit = 0;

        // curve kick planning
      } else {
        reset_target = false;
        if (target_angle == -10) {
          if (goal.visible) {
            target_angle = goal.relative_angle + robot.gyro_angle;
          } else {
            Vec target{91, 237};
            Vec route = target - robot.position;
            target_angle =
                route.field_angle() - robot.field_angle + robot.gyro_angle;
          }
        }

        double delta = normalize_angle(target_angle - robot.gyro_angle);

        if (abs(delta) <= 0.1) {
          if (rotated_tm == -1) {
            rotated_tm = millis();
          }
          if (millis() - rotated_tm >= 400) {
            cout << "FIRE" << endl;
            robot.kicker_force = 60;  // здесь было 100
            robot.rotation = 0;
            // throttle = millis() + 1000;
            fired = millis();
          } else {
            robot.dribling = 0;
            robot.vel = robot.vel.resize(0);
            cout << "WAIT" << endl;
          }
        } else {
          cout << "ROTATION " << abs(delta) << endl;
          robot.dribling = 60;
          robot.vel = robot.vel.resize(0);
          robot.rotation = target_angle - robot.gyro_angle;
          robot.rotation_limit = 10;
        }
      }
    }
  } else {
    Vec target{91, 121};
    robot.vel = target - robot.position;
    robot.vel = robot.vel.resize(min(robot.vel.len(), 40.0));
    robot.rotation = 0;
    robot.rotation_limit = 30;
  }
  robot.vel = robot.vel.resize(min(robot.vel.len(), 80.0));
  if (reset_target) {
    target_angle = -10;
    rotated_tm = -1;
  }
}