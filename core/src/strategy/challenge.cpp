#include "strategy/strategy.h"
#include "utils/millis.h"

static long long start_tm = -1;

void Strategy::run_challenge(Robot& robot, Object& ball, Object& goal) {
  cout << robot.gyro_angle << endl;
  if (start_tm == -1) {
    start_tm = millis();
  }
  if (start_tm + 3000 + 1000 <= millis()) {
    robot.state = RobotState::PAUSE;
    start_tm = -1;
  } else if (start_tm + 3000 <= millis()) {
    robot.dribling = 30;
    robot.rotation_limit = 50;
    robot.rotation = normalize_angle(M_PI - robot.field_angle);
  } else {
    robot.dribling = 100;
  }
  // robot.dribling = max_dribling;
  // double dir = M_PI / 2;
  // Vec vel{robot.field_angle};
  // vel = vel.turn_right();
  // vel = vel.rotate(0.1);
  // vel = vel.resize(17);
  // robot.vel = vel;
  // robot.rotation_limit = 15;
  // robot.rotation = dir;
}