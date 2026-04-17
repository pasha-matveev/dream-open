#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>

#include "utils/config.h"
#include "utils/geo/vec.h"
#include "utils/millis.h"

Strategy::Strategy() { role = config.strategy.role; }

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

  if (config.serial.enabled) {
    robot.compute_gyro_angle();
  }
  // Всегда двигаем позу предсказанием; лидар потом её уточняет EMA-блендингом.
  if (config.serial.enabled) {
    robot.predict_position(dt);
  }
  if (auto measured = robot.compute_lidar(); measured.has_value()) {
    if (has_lidar_fix) {
      // Экспоненциальное сглаживание позы с лидаром.
      static constexpr double ALPHA_XY = 0.3;
      static constexpr double ALPHA_ANG = 0.5;
      robot.position =
          measured->position * ALPHA_XY + robot.position * (1.0 - ALPHA_XY);
      double diff = normalize_angle(measured->field_angle - robot.field_angle);
      robot.field_angle = normalize_angle(robot.field_angle + ALPHA_ANG * diff);
    } else {
      // Первый кадр — хард-снап, предсказания ещё нет.
      robot.position = measured->position;
      robot.field_angle = measured->field_angle;
      has_lidar_fix = true;
    }
  }
  if (config.visualization.interactive) {
    // В интерактивной визуализации ball.field_position / ball.relative_angle
    // ставятся через мышь в visualization.cpp и ball.visible поднимается там
    // же. Здесь только пробрасываем состояние в last_ball_*.
    if (ball.visible) {
      last_ball_visible = millis();
      last_ball_position = ball.field_position;
      last_ball_relative_angle = ball.relative_angle;
    }
  } else if (robot.camera->new_data()) {
    ball.compute_field_position(robot);
    if (ball.visible) {
      assert(ball.field_position.x >= 0 && ball.field_position.y >= 0);
      last_ball_visible = millis();
      last_ball_position = ball.field_position;
      last_ball_relative_angle = ball.relative_angle;
    }
  }

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

  if (config.strategy.enabled) {
    reset_kick = true;
    reset_turn = true;

    if (robot.state == RobotState::RUNNING) {
      if (role == "attacker") {
        run_attacker(robot, ball, goal, field);
      } else if (role == "keeper") {
        run_keeper(robot, ball, goal, field);
      } else if (role == "challenge") {
        run_challenge(robot, ball, goal);
      } else if (role == "test_circle") {
        run_test_circle(robot);
      } else if (role == "test_dribling") {
        run_test_dribling(robot);
      } else if (role == "test") {
        run_test(robot, goal);
      } else {
        spdlog::error("Unknown role: {}", role);
      }
    }

    if (reset_kick) {
      kick_status = KickStatus::NONE;
      kick_timeout_stamp = -10000;
    }
    if (reset_turn) {
      turn_start_time = -1;
    }
    field.apply(robot);
  }
  robot.prev_emitter = robot.emitter;
  last_dubins = cur_dubins;
  cur_dubins = false;

  robot.apply_motion_limits(dt);
}
