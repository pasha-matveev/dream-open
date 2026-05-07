#pragma once

#include "robot.h"
#include "utils/geo/vec.h"

bool drive_target(Robot& robot, const Vec& target, double max_speed = 120,
                  double min_speed = 0, bool is_ball = false);
void drive_ball(Robot& robot, const Vec& ball);
void accelerated_dribbling(Robot& robot);
void desired_dribbling(Robot& robot, bool ac_dribbling);
bool take_ball(Robot& robot, long long forward_timeout);
double compute_ricochet(Robot& robot, bool left);
