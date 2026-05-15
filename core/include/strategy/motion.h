#pragma once

#include "robot.h"
#include "utils/geo/vec.h"

struct Mapper;

struct StabilizeCaptureParams {
  const Mapper& dribbling;
  double initial_speed = 10.0;
};

bool drive_target(Robot& robot, const Vec& target, double max_speed = 120,
                  double min_speed = 0, bool is_ball = false,
                  const Mapper* speed_map = nullptr);
void drive_ball(Robot& robot, const Vec& ball);
void accelerated_dribbling(Robot& robot, const Mapper& dribbling);
bool stabilize_capture(Robot& robot, const StabilizeCaptureParams& params);
double compute_ricochet_field_angle(const Vec& ball_pos, const Vec& target,
                                    bool left);
