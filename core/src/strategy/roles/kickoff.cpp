#include <spdlog/spdlog.h>

#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/kickoff.h"
#include "strategy/motion.h"
#include "strategy/strategy.h"
#include "strategy/turn.h"
#include "utils/geo/vec.h"
#include "utils/millis.h"

namespace {

// Архитектура — тот же паттерн, что в attacker.cpp со SpinPipelineState:
// state живёт как function-static, никому за пределами run_kickoff не нужен.
enum class KickoffPhase {
  CAPTURE_BLIND,
  STABILIZE,
  TURN_TO_RICOCHET,
  KICK,
  DONE
};

struct KickoffPipelineState {
  KickoffPhase phase = KickoffPhase::CAPTURE_BLIND;
  // Стартовое время текущей фазы (для CAPTURE_BLIND и KICK). -1 = ещё не вошли.
  long long phase_start_ms = -1;
  // Сторона разводки. Запоминаем при сбросе пайплайна, чтобы дальше не зависеть
  // от robot.state (его может изменить PAUSE-кнопка).
  bool left = false;
  // Целевой field_angle, вычисленный один раз в TURN_TO_RICOCHET.
  double target_field_angle = 0;
  bool target_computed = false;
  // Стартовое время фазы KICK (для kick_followthrough_ms).
  long long kick_start_ms = -1;
};

}  // namespace

void Strategy::run_kickoff(Robot& robot, Object& ball, Object& goal,
                           Field& field, bool left) {
  static KickoffPipelineState s;

  // Сброс при ЛЮБОМ нажатии KICKOFF-кнопки. Кнопка нажимается из ISR-потока,
  // поэтому передаём сигнал через atomic-флаг robot.kickoff_reset_request.
  // Button handler set-ает; здесь exchange(false) атомарно прочитал-и-сбросил.
  // Это покрывает оба сценария:
  //   1) первый запуск разводки (флаг выставлен в handle_*_button);
  //   2) PAUSE посреди разводки → второе нажатие KICKOFF → флаг снова true.
  // На случай если флаг каким-то образом не выставлен, fallback на phase==DONE.
  if (robot.kickoff_reset_request.exchange(false) ||
      s.phase == KickoffPhase::DONE) {
    s = {};
    s.left = left;
  }

  const auto& cfg_k = *config->strategy->kickoff;

  switch (s.phase) {
    case KickoffPhase::CAPTURE_BLIND: {
      spdlog::info("KICKOFF CAPTURE_BLIND");
      if (s.phase_start_ms < 0) s.phase_start_ms = millis();
      long long elapsed = millis() - s.phase_start_ms;

      if (robot.emitter) {
        // Поймали мяч — идём в стандартную стабилизацию.
        s.phase = KickoffPhase::STABILIZE;
        break;
      }
      if (elapsed >= cfg_k.capture_blind_timeout_ms) {
        // Не поймали — STABILIZE опирается на robot.first_time (rising-edge
        // emitter), без захвата он stale. Пропускаем стабилизацию и сразу
        // идём поворачиваться + бить «в воздух». Робот после этого вернётся
        // в RUNNING и attacker подберёт мяч обычным путём.
        spdlog::warn("KICKOFF capture timeout, skipping STABILIZE");
        s.phase = KickoffPhase::TURN_TO_RICOCHET;
        break;
      }

      // Идентично take_ball, но скорость и таймаут управляются конфигом
      // разводки, а не общим dribbling.param_r.
      double speed = cfg_k.capture_speed *
                     (1.0 - double(elapsed) / cfg_k.capture_blind_timeout_ms);
      robot.vel = Vec{robot.field_angle}.resize(speed);
      accelerated_dribbling(robot);
      robot.rotation = 0;
      robot.rotation_limit = 0;
      break;
    }

    case KickoffPhase::STABILIZE: {
      spdlog::info("KICKOFF STABILIZE");
      // Сюда попадаем только если в CAPTURE_BLIND emitter сработал, поэтому
      // robot.first_time гарантированно валиден (выставлен в strategy.cpp
      // на rising edge emitter).
      if (!stabilize_capture(robot)) {
        s.phase = KickoffPhase::TURN_TO_RICOCHET;
      }
      break;
    }

    case KickoffPhase::TURN_TO_RICOCHET: {
      spdlog::info("KICKOFF TURN_TO_RICOCHET");
      if (!s.target_computed) {
        Vec target = s.left ? Vec{cfg_k.target_left.x, cfg_k.target_left.y}
                            : Vec{cfg_k.target_right.x, cfg_k.target_right.y};
        s.target_field_angle =
            compute_ricochet_field_angle(robot.ball_hole_position(), target,
                                         s.left);
        s.target_computed = true;
        spdlog::info("KICKOFF ricochet target_field_angle={:.3f}",
                     s.target_field_angle);
      }
      bool finished = turn_->execute(
          robot, {.target_field_angle = s.target_field_angle,
                  .curved_rotation = false,
                  .accelerated_dribbling = true});
      if (finished) {
        s.phase = KickoffPhase::KICK;
      }
      break;
    }

    case KickoffPhase::KICK: {
      spdlog::info("KICKOFF KICK");
      if (s.kick_start_ms < 0) s.kick_start_ms = millis();
      // Стреляем кикером — Arduino считывает kicker_force каждый тик;
      // выставляем его и ждём короткий «после-удар» таймаут.
      robot.kicker_force = cfg_k.kick_force;
      robot.dribbling = 0;
      robot.vel = {0, 0};
      robot.rotation = 0;
      if (millis() - s.kick_start_ms >= cfg_k.kick_followthrough_ms) {
        s.phase = KickoffPhase::DONE;
      }
      break;
    }

    case KickoffPhase::DONE: {
      spdlog::info("KICKOFF DONE");
      // Используем существующий механизм Strategy::stop_until — он держит
      // vel=0 / rotation=0 до момента millis() >= stop_until (см. strategy.cpp).
      // Это даёт мячу время улететь из дриблера до того, как обычная роль
      // (attacker) возьмёт управление.
      stop_until = millis() + cfg_k.post_kickoff_delay_ms;
      robot.state = RobotState::RUNNING;
      // s сбросится при следующем входе в run_kickoff (по флагу или phase==DONE).
      break;
    }
  }
}
