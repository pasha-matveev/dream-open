#include <cmath>

#include "strategy/strategy.h"
#include "utils/millis.h"

static long long kickoff_start = -1;
static bool shot = false;
static bool fw = false;
static bool has_target = false;
static double target = 0;

static void finish(Robot& robot) {
  robot.state = RobotState::PAUSE;
  kickoff_start = -1;
  shot = false;
  fw = false;
  has_target = false;
  target = 0;
}

void Strategy::run_kickoff(Robot& robot, Object& ball, Object& goal,
                           bool left) {
  if (kickoff_start == -1) {
    kickoff_start = millis();
  }
  long long elapsed = millis() - kickoff_start;
  if (elapsed > 2000) {
    cout << "KICKOFF TIMEOUT " << endl;
    finish(robot);
    return;
  }
  if (robot.emitter) {
    // Мячик в лунке, бьем
    robot.kicker_force = 70;
    shot = true;
  } else {
    if (shot) {
      // Уже ударили и мячик улетел
      finish(robot);
      return;
    }
    // Нужно ровно подъехать к мячу
    if (!has_target) {
      has_target = true;
      // double alpha = atan(91.0 / 60.75);
      double alpha = 1.0;
      if (left) {
        target = robot.gyro_angle - robot.field_angle + alpha;
      } else {
        target = robot.gyro_angle - robot.field_angle - alpha;
      }
    }

    double delta = normalize_angle(target - robot.gyro_angle);
    cout << target << " " << delta << endl;

    if (abs(delta) <= 0.05 || fw) {
      // Повернулись, едем вперед
      fw = true;
      robot.rotation = delta;

      Vec vel{M_PI / 4};
      vel = vel.resize(20);
      robot.vel = vel;
    } else {
      // Поворачиваемся
      Vec vel{robot.field_angle};
      vel = vel.turn_right();

      vel = vel.resize(50);
      double max_rotation = 30;
      robot.rotation_limit =
          min(max_rotation, (30 - 10) * (50.0 / elapsed) + 10);
      robot.rotation = (delta < 0) ? -M_PI / 2 : M_PI / 2;
      robot.vel = vel;
    }
  }
}