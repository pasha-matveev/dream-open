#pragma once

#include <deque>
#include <optional>
#include <utility>

#include "utils/geo/vec.h"

// VigilansController — режим бдительности вратаря.
//
// Когда мяч долго лежит без движения и рядом нет препятствий, вратарь в обход
// обычной логики едет к мячу и бьёт по чужим воротам (актуально, когда обоих
// нападающих удалили с поля).
//
// Детекция покоя работает по СЫРЫМ позициям мяча (ball_filter на роботах
// выключен): храним deque позиций за последние still_ms и считаем мяч
// неподвижным, когда все позиции окна лежат в радиусе still_radius от центроида
// окна, и окно набрано целиком (трекинг идёт непрерывно >= still_ms).
//
// FSM:
//
//                  enabled · cooldown_passed · ball.y<=ball_max_y
//                  · stationary(still_ms, центроид±still_radius) ·
//                  obstacle_clear
//    IDLE ----------------------------------------------------------> ACTIVE
//     ^   (observe() копит deque позиций каждый тик)                    |
//     |                                       emitter:  execute_to_goal |
//     |                                      !emitter:  drive_ball      |
//     |                                                                 |
//     +------- COOLDOWN <-----------------------------------------------+
//               (last_deactivated_at_)   причины выхода:
//                                        - kick: emitter спал (фронт)
//                                        - timeout: прошло >= timeout_ms
//                                        - lost: мяч не виден > lost_ms (drive)
//
// Конфиг (config->strategy->keeper->vigilans) читается напрямую внутри методов,
// как KickController/BallTracker читают глобальный config.
class VigilansController {
 public:
  // Сэмплирование позиции мяча каждый тик. opt_pos == std::nullopt — мяч не
  // свежий, окно покоя сбрасывается.
  void observe(const std::optional<Vec>& opt_pos, long long now);

  // Мяч непрерывно лежит неподвижно полное окно still_ms.
  bool stationary(long long now) const;

  bool active() const { return active_; }
  long long activated_at() const { return activated_at_; }
  bool cooldown_passed(long long now) const;
  void activate(long long now);
  void deactivate(long long now);
  void clear();

  // Хранение emitter прошлого тика для детекции удара по падающему фронту.
  bool had_emitter() const { return had_emitter_; }
  void set_emitter(bool e) { had_emitter_ = e; }

 private:
  std::deque<std::pair<long long, Vec>> history_;
  // Момент начала непрерывной видимости (для гарантии полного окна).
  long long continuous_since_ = 0;
  bool active_ = false;
  long long activated_at_ = 0;
  // Отрицательное стартовое значение, чтобы первая активация разрешалась сразу.
  long long last_deactivated_at_ = -100000;
  bool had_emitter_ = false;
};
