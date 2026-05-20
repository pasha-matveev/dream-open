#pragma once

#include "robot.h"
#include "utils/geo/vec.h"

struct Mapper;
class KickController;

struct StabilizeCaptureParams {
  const Mapper& dribbling;
  double initial_speed = 10.0;
};

struct DriveParams {
  double max_speed = 120;
  double min_speed = 0;
  bool is_ball = false;
  // Своя кривая дистанция→скорость (напр. fast_direct). nullptr — берётся
  // control->speed. Учитывается только при is_ball.
  const Mapper* speed_map = nullptr;
  // true  — скорость ограничена физическим пределом торможения до остановки
  //         в цели (v_safe = sqrt(2·a·d));
  // false — предел снят, скорость задаёт только маппер / max_speed (таран).
  bool brake_safe = true;
  double prec = 2;
};

bool drive_target(Robot& robot, const Vec& target,
                  const DriveParams& params = {});
void drive_ball(Robot& robot, const Vec& ball);
void accelerated_dribbling(Robot& robot, const Mapper& dribbling);
bool stabilize_capture(Robot& robot, const StabilizeCaptureParams& params);
double compute_ricochet_field_angle(const Vec& ball_pos, const Vec& target,
                                    bool left);

// Повернуться на угол рикошета от ближней стенки и ударить. При
// recompute_each_tick == true угол пересчитывается каждый вызов от текущего
// ball_hole_position; иначе фиксируется при первом вызове в cached_field_angle
// (angle_computed — флаг latch, владелец хранит снаружи).
void execute_ricochet_kick(Robot& robot, KickController& kick, bool left,
                           const Vec& target, bool recompute_each_tick,
                           bool curved_rotation,
                           const Mapper* dribbling_slowdown,
                           double& cached_field_angle, bool& angle_computed);
