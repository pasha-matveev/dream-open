#include "strategy/hobubu.h"

#include <spdlog/spdlog.h>

#include <cmath>

#include "config/strategy/attacker.h"
#include "robot.h"
#include "strategy/accel_drive.h"
#include "strategy/kurwa.h"
#include "strategy/motion.h"
#include "tracking/object.h"
#include "utils/geo/vec.h"

bool HobubuController::execute(Robot& robot, Object& goal, bool use_left_hint,
                               const cfg::Attacker& acfg) {
  reset_pending_ = false;

  if (phase_ == Phase::IDLE) {
    use_left_ = use_left_hint;
    phase_ = Phase::FACE_WALL;
  }

  const auto& cfg = *acfg.hobubu;
  double sign = use_left_ ? +1.0 : -1.0;
  double face_target = sign * cfg.face_wall_angle;
  // Прямая проезда: на расстоянии wall_dist от ближнего бортика.
  double line_x = use_left_ ? cfg.wall_dist : 182.0 - cfg.wall_dist;
  // Верх проезда — широта pos_1 у kurwa (та же конвенция 240 - y). Дальше
  // kurwa сама дотянет мяч к pos_1 как обычно.
  double top_y = 240.0 - acfg.kurwa->pos_1->y;

  robot.dribbling = cfg.dribbling;

  switch (phase_) {
    case Phase::IDLE:
      // Unreachable: IDLE переведён в FACE_WALL выше.
      break;
    case Phase::FACE_WALL: {
      spdlog::info("HOBUBU FACE_WALL");
      double delta = normalize_angle(face_target - robot.field_angle);
      robot.vel = {0, 0};
      robot.rotation = delta;
      if (std::abs(delta) < cfg.ready_angle) {
        phase_ = Phase::DRIVE_TO_LINE;
      }
      break;
    }
    case Phase::DRIVE_TO_LINE: {
      spdlog::info("HOBUBU DRIVE_TO_LINE");
      // Подъезд вбок к прямой (цель по y = текущая y, движемся только по x),
      // без accel-разгона. Удерживаем ориентацию к стенке.
      drive_target(robot, {line_x, robot.position.y},
                   {.max_speed = cfg.drive_max_speed});
      robot.rotation = normalize_angle(face_target - robot.field_angle);
      if (std::abs(robot.position.x - line_x) < cfg.ready_dist) {
        phase_ = Phase::DRIVE_UP;
      }
      break;
    }
    case Phase::DRIVE_UP: {
      spdlog::info("HOBUBU DRIVE_UP");
      // Едем вверх вдоль борта с плавным разгоном.
      AccelDriveParams p;
      p.target = {line_x, top_y};
      p.max_speed = cfg.drive_max_speed;
      accel_drive_->execute(robot, p);
      robot.rotation = normalize_angle(face_target - robot.field_angle);
      if (robot.position.y >= top_y - cfg.ready_dist) {
        phase_ = Phase::KURWA;
      }
      break;
    }
    case Phase::KURWA: {
      // Финал как обычно — делегируем kurwa с зафиксированной стороной.
      bool done = kurwa_->execute(robot, goal, use_left_, acfg);
      if (done) {
        phase_ = Phase::IDLE;
        return true;
      }
      break;
    }
  }

  return false;
}
