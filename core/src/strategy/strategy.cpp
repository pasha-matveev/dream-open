#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>

#include "strategy/ball_tracker.h"
#include "strategy/dubins.h"
#include "strategy/kick.h"
#include "strategy/turn.h"
#include "config/config.h"
#include "config/serial.h"
#include "config/strategy.h"
#include "config/visualization.h"
#include "utils/geo/vec.h"
#include "utils/millis.h"

Strategy::Strategy() {
  // Фаза 1: создаём все контроллеры без зависимостей. Порядок не важен —
  // ни один контроллер не трогает другой в конструкторе.
  ball_ = std::make_unique<BallTracker>();
  turn_ = std::make_unique<TurnController>();
  kick_ = std::make_unique<KickController>();
  dubins_ = std::make_unique<DubinsController>();

  // Фаза 2: связываем зависимости явно. Циклы между контроллерами теперь
  // тоже возможны (если вдруг понадобятся в будущем).
  kick_->init(turn_.get());
  dubins_->init(kick_.get(), ball_.get());

  role = config->strategy->role;
}

Strategy::~Strategy() = default;

void Strategy::run(Robot& robot, Object& ball, Object& goal, Field& field) {
  long long now = millis();
  double dt = 0.0;
  if (last_tick_ms >= 0) {
    dt = std::clamp((now - last_tick_ms) / 1000.0, 0.0, 0.1);
  }
  last_tick_ms = now;
  last_dt = dt;

  if (dt > 0) {
    if (avg_dt == 0.0) {
      avg_dt = dt;
    } else {
      avg_dt = 0.9 * avg_dt + 0.1 * dt;
    }
  }
  if (now - last_fps_log_ms >= 1000) {
    last_fps_log_ms = now;
    if (avg_dt > 0) {
      spdlog::info("Strategy FPS: {:.1f}", 1.0 / avg_dt);
    }
  }

  // Всегда двигаем позу предсказанием; лидар потом её уточняет EMA-блендингом.
  if (config->serial->enabled) {
    robot.compute_gyro_angle();
    robot.predict_position(dt);
  }
  if (auto measured = robot.compute_lidar(); measured.has_value()) {
    if (has_lidar_fix) {
      // Позицию сглаживаем EMA с лидаром. Угол оставляем от гироскопа —
      // он точнее короткосрочно; долгосрочный дрейф правит calibrate() по
      // стабильной позе внутри compute_lidar.
      robot.position =
          measured->position * config->strategy->predict->alpha_xy +
          robot.position * (1.0 - config->strategy->predict->alpha_xy);
    } else {
      // Первый кадр — хард-снап позиции и синхронизация гироскопа к лидару.
      robot.position = measured->position;
      robot.calibrate(measured->field_angle);
      robot.compute_gyro_angle();
      has_lidar_fix = true;
    }
  }
  ball_->update(robot, ball, dt, now, config->visualization->interactive);

  if (robot.emitter && !robot.prev_emitter) {
    robot.first_time = millis();
  }

  if (millis() < stop_until) {
    robot.vel = {0, 0};
    robot.rotation = 0;
    robot.rotation_limit = 0;
    robot.apply_motion_limits(dt);
    return;
  }

  robot.kicker_force = 0;
  robot.dribling = 0;
  robot.rotation = 0;
  robot.vel = {0, 0};
  robot.rotation_limit = 50;

  if (config->strategy->enabled) {
    kick_->mark_for_reset();
    turn_->mark_for_reset();

    if (robot.state == RobotState::RUNNING) {
      if (role == "attacker") {
        run_attacker(robot, ball, goal, field);
      } else if (role == "keeper") {
        run_keeper(robot, ball, goal, field);
      } else if (role == "challenge") {
        run_challenge(robot, ball, goal);
      } else if (role == "test_circle") {
        run_test_circle(robot);
      } else if (role == "test_dribbling") {
        run_test_dribling(robot);
      } else if (role == "test_mirror") {
        run_test_mirror(robot, ball);
      } else if (role == "test") {
        run_test(robot, goal);
      } else if (role == "test_emitter") {
        run_test_emitter(robot);
      } else {
        spdlog::error("Unknown role: {}", role);
      }
    }

    kick_->apply_reset_if_pending();
    turn_->apply_reset_if_pending();
    field.apply(robot);
  }
  dubins_->on_tick_end();

  robot.apply_motion_limits(dt);
}
