#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <vector>

#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/line.h"
#include "field_dims.h"
#include "robot.h"
#include "tracking/camera.h"
#include "utils/geo/vec.h"
#include "utils/millis.h"

namespace {
// Файловое состояние режима (как square_state в roles/test.cpp).
// Время последнего кадра, где линия была видна (мс); -1 — ещё ни разу.
long long last_good_ms = -1;
// Throttle debug-лога.
long long last_log_ms = -1;
}  // namespace

// Body-frame визуальный серво по линии. Каждый тик берём снимок углов
// пересечений кольца с линией (в кадре робота), выбираем ближайший к «вперёд»
// (θ=0) в конусе max_turn_deg, едем туда и доворачиваем нос. Без абсолютной
// памяти курса — поэтому дрейф гироскопа не накапливается (конверсия body→field
// каждый тик сокращается: vel.field_angle() - field_angle == θ).
void Strategy::run_line(Robot& robot) {
  const cfg::Line& lc = *config->strategy->line;

  // Пин позиции в центр поля: гасит дрейф predict_position и не даёт
  // resolve_rotation() включить safe_turn (он гейтится position.y <
  // safe_turn.y; центр поля выше порога). Настоящая позиция тут не нужна.
  robot.position = field_dims::kCenter;
  robot.rotation_limit = lc.rotation_limit;

  std::vector<double> cand;
  if (robot.camera) cand = robot.camera->get_line_candidates();

  const long long now = millis();
  const double max_turn = lc.max_turn_deg * M_PI / 180.0;

  // Ближайший к «вперёд» кандидат в конусе. Это автоматически отсекает обратное
  // пересечение (~180°) и слишком резкие срывы.
  bool found = false;
  double best_theta = 0.0, best_abs = max_turn;
  for (double theta : cand) {
    const double a = std::abs(normalize_angle(theta));
    if (a <= best_abs) {
      best_abs = a;
      best_theta = theta;
      found = true;
    }
  }

  if (found) {
    last_good_ms = now;
    robot.vel =
        Vec{normalize_angle(robot.field_angle + best_theta)}.resize(lc.speed);
    robot.rotation = lc.rotation_gain * best_theta;
  } else if (last_good_ms >= 0 && now - last_good_ms < lc.lost_timeout_ms) {
    // Линия потеряна недолго — коастим прямо по носу, без доворота.
    robot.vel = Vec{robot.field_angle}.resize(lc.speed);
    robot.rotation = 0;
  } else {
    // Совсем потеряли — стоп.
    robot.vel = {0, 0};
    robot.rotation = 0;
  }

  const char* state =
      found ? "FOLLOW"
            : (last_good_ms >= 0 && now - last_good_ms < lc.lost_timeout_ms)
                  ? "COAST"
                  : "STOP";
  if (last_log_ms < 0 || now - last_log_ms >= 500) {
    last_log_ms = now;
    spdlog::info("line: cand={} state={} theta={:.3f}", cand.size(), state,
                 found ? best_theta : 0.0);
  }
}
