#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>

#include "config/config.h"
#include "config/lidar.h"
#include "config/serial.h"
#include "config/strategy.h"
#include "config/strategy/control.h"
#include "config/visualization.h"
#include "strategy/accel_drive.h"
#include "strategy/ball_tracker.h"
#include "strategy/dubins.h"
#include "strategy/kick.h"
#include "strategy/motion.h"
#include "strategy/spin_shot.h"
#include "strategy/test.h"
#include "strategy/turn.h"
#include "utils/geo/vec.h"
#include "utils/millis.h"

Strategy::Strategy() {
  // Фаза 1: создаём все контроллеры без зависимостей. Порядок не важен —
  // ни один контроллер не трогает другой в конструкторе.
  ball_ = std::make_unique<BallTracker>();
  turn_ = std::make_unique<TurnController>();
  kick_ = std::make_unique<KickController>();
  dubins_ = std::make_unique<DubinsController>();
  spin_shot_ = std::make_unique<SpinShotController>();
  accel_drive_ = std::make_unique<AccelDriveController>();
  test_ = std::make_unique<TestController>();

  // Фаза 2: связываем зависимости явно. Циклы между контроллерами теперь
  // тоже возможны (если вдруг понадобятся в будущем).
  kick_->init(turn_.get());
  dubins_->init(kick_.get(), ball_.get());

  role = config->strategy->role;

  // Ускорение с мячом должно быть не больше обычного ускорения
  if (config->strategy->control->ball_drive->max_accel >=
      config->strategy->motion->max_linear_accel) {
    spdlog::error(
        "ball_drive.max_accel ({}) >= motion.max_linear_accel ({}); "
        "global slew will dominate, local cap has no effect",
        config->strategy->control->ball_drive->max_accel,
        config->strategy->motion->max_linear_accel);
  }
}

Strategy::~Strategy() = default;

void Strategy::run(Robot& robot, Object& ball, Object& goal, Object& own_goal,
                   Field& field) {
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

  // В PAUSE моторы стоят — обнуляем actual_vel, чтобы predict_position
  // не сдвигал позицию во время калибровки по своим воротам.
  if (robot.state == RobotState::PAUSE) {
    robot.actual_vel = {0, 0};
  }

  // Всегда двигаем позу предсказанием; лидар потом её уточняет EMA-блендингом.
  if (config->serial->enabled) {
    robot.compute_gyro_angle();
    robot.predict_position(dt);
  }

  // PAUSE-разворот по своим воротам: лидар не различает 180°, поэтому в PAUSE
  // используем свои ворота для определения, в какой половине мы стоим.
  // Гистерезис: переключаем калибровку только если current half отличается
  // от target — чтобы не затирать точную подстройку гироскопа лидаром.
  // Если свои ворота не видны дольше own_goal_timeout, считаем что робот
  // смотрит на ворота противника (field_angle=0).
  if (robot.state == RobotState::PAUSE) {
    if (own_goal.camera_visible) {
      last_own_goal_seen_ms = now;
      double a = own_goal.relative_angle;
      double target_half = (abs(normalize_angle(a)) < M_PI / 2) ? M_PI : 0.0;
      // Текущая половина гироскопа.
      double current_half =
          (abs(normalize_angle(robot.field_angle)) < M_PI / 2) ? 0.0 : M_PI;
      if (abs(normalize_angle(current_half - target_half)) > M_PI / 2) {
        robot.calibrate(target_half);
        robot.compute_gyro_angle();
      }
    } else if (last_own_goal_seen_ms >= 0 &&
               now - last_own_goal_seen_ms >
                   config->lidar->calibration->own_goal_timeout) {
      // Долго не видим свои ворота — fallback к ориентации на чужие.
      if (abs(normalize_angle(robot.field_angle)) > M_PI / 2) {
        robot.calibrate(0.0);
        robot.compute_gyro_angle();
      }
    }
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
      // Первый кадр — хард-снап позиции и синхронизация гироскопа к лидару
      // (точная подстройка половины, выбранной выше через свои ворота).
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
  robot.prev_emitter = robot.emitter;

  robot.kicker_force = 0;
  robot.dribbling = 0;
  robot.rotation = 0;
  robot.vel = {0, 0};
  robot.rotation_limit = 15;

  if (millis() < stop_until) {
    robot.rotation_limit = 0;
    robot.apply_motion_limits(dt);
    return;
  }

  if (config->strategy->enabled) {
    kick_->mark_for_reset();
    turn_->mark_for_reset();
    spin_shot_->mark_for_reset();
    accel_drive_->mark_for_reset();

    if (robot.state == RobotState::RUNNING) {
      if (role == "attacker") {
        run_attacker(robot, ball, goal, field);
      } else if (role == "keeper") {
        run_keeper(robot, ball, goal, field);
      } else if (role == "challenge") {
        run_challenge(robot, ball, goal);
      } else {
        TestContext ctx{robot, ball, goal, *ball_};
        if (!test_->run(role, ctx)) {
          spdlog::error("Unknown role: {}", role);
        }
      }
    } else if (robot.state == RobotState::KICKOFF_LEFT ||
               robot.state == RobotState::KICKOFF_RIGHT) {
      bool left = robot.state == RobotState::KICKOFF_LEFT;
      run_kickoff(robot, ball, goal, field, left);
    }

    kick_->apply_reset_if_pending();
    turn_->apply_reset_if_pending();
    spin_shot_->apply_reset_if_pending();
    accel_drive_->apply_reset_if_pending();
    field.apply(robot);
  }
  dubins_->on_tick_end();

  robot.apply_motion_limits(dt);
}
