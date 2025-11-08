#include <cmath>

#include "strategy/strategy.h"
#include "utils/millis.h"

static long long kickoff_start = -1;
static bool shot = false;
static bool fw = false;
static bool has_target = false;
static double target = 0;
static long long go_time = -1;

static void finish(Robot& robot) {
  robot.state = RobotState::PAUSE;
  kickoff_start = -1;
  shot = false;
  fw = false;
  has_target = false;
  target = 0;
  go_time = -1;
}

void Strategy::run_kickoff(Robot& robot, Object& ball, Object& goal,
                           bool left) {
  if (kickoff_start == -1) {
    kickoff_start = millis();
  }
  long long elapsed = millis() - kickoff_start;
  if (elapsed > 5000) {
    cout << "KICKOFF TIMEOUT " << endl;
    finish(robot);
    return;
  }
  if (robot.emitter) {
    // 1.7
    Vec ball_position = robot.position + Vec{robot.field_angle}.resize(9.5);
    Vec target{91, 230};
    double a = ball_position.x;
    double b = 230 - ball_position.y;
    double c = b / 2;
    double alpha = atan(a / c);
    double s = (left) ? 1 : -1;
    cout << robot.field_angle << endl;
    kick_dir(robot, alpha * s - robot.field_angle, 100, 600, true, 0, 0.01);
  } else {
    robot.vel = Vec{robot.field_angle + ball.relative_angle}.resize(5);
    robot.rotation = ball.relative_angle;
    robot.dribling = base_dribling;
  }

  // if (robot.emitter && go_time != -1 && go_time <= millis()) {
  //   // Мячик в лунке, бьем
  //   robot.kicker_force = 70;
  //   shot = true;
  // } else {
  //   if (shot) {
  //     // Уже ударили и мячик улетел
  //     finish(robot);
  //     return;
  //   }
  //   // Нужно ровно подъехать к мячу
  //   if (!has_target) {
  //     has_target = true;
  //     // double alpha = atan(91.0 / 60.75);
  //     double alpha = 1.0;
  //     if (left) {
  //       target = robot.gyro_angle - robot.field_angle + alpha;
  //     } else {
  //       target = robot.gyro_angle - robot.field_angle - alpha;
  //     }
  //   }

  //   double delta = normalize_angle(target - robot.gyro_angle);

  //   if (abs(delta) <= 0.05 || fw) {
  //     // Повернулись, едем вперед
  //     fw = true;

  //     if (go_time == -1) {
  //       go_time = millis() + 30;
  //     }
  //     Vec vel;
  //     if (go_time <= millis()) {
  //       robot.rotation = ball.relative_angle;
  //       vel = Vec{ball.relative_angle + robot.field_angle};
  //       vel = vel.resize(15);
  //     } else {
  //       robot.rotation = delta;
  //       // vel = Vec{alpha};
  //       vel = {0, 0};
  //     }
  //     robot.rotation_limit = 30;
  //     robot.vel = vel;
  //   } else {
  //     // Поворачиваемся
  //     Vec vel{robot.field_angle};
  //     vel = vel.turn_right();

  //     vel = vel.resize(45);
  //     vel = vel.rotate(0.18);
  //     // 50 30 5 80
  //     double ma = 30;
  //     double mi = 5;
  //     double duration = 80;
  //     robot.rotation_limit = min(ma, (ma - mi) * (elapsed / duration) + mi);
  //     robot.rotation = M_PI / 2;
  //     robot.vel = vel;
  //   }
  // }
}