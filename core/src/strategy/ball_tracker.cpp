#include "strategy/ball_tracker.h"

#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/ball_filter.h"
#include "robot.h"

void BallTracker::update(Robot& robot, Object& ball, double dt, long long now,
                         bool interactive_mode) {
  if (interactive_mode) {
    // В интерактивной визуализации ball.field_position / ball.relative_angle
    // ставятся через мышь в visualization.cpp и ball.visible поднимается там
    // же. Здесь только пробрасываем состояние в last_position_. Фильтр в этом
    // режиме не работает: позиция от мыши — идеальная "истина", фильтр её
    // только исказит.
    if (ball.visible) {
      last_visible_ = now;
      last_position_ = ball.field_position;
    }
    return;
  }

  if (config->strategy->ball_filter->enabled) {
    filter_.predict(dt);
  }
  if (robot.camera && robot.camera->new_data()) {
    ball.compute_field_position(robot);
    if (ball.visible) {
      last_visible_ = now;
      if (config->strategy->ball_filter->enabled) {
        filter_.update(ball.field_position, now);
      }
    }
  }
  if (config->strategy->ball_filter->enabled) {
    if (!ball.visible) {
      filter_.lost(now);
    }
    if (filter_.is_initialized()) {
      last_position_ = filter_.position();
    }
  } else {
    last_position_ = ball.field_position;
  }
}

double BallTracker::relative_angle(const Robot& robot) const {
  return normalize_angle((last_position_ - robot.position).field_angle() -
                         robot.field_angle);
}
