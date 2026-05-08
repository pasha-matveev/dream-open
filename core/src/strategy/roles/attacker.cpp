#include <spdlog/spdlog.h>

#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/attacker.h"
#include "config/strategy/dubins.h"
#include "strategy/ball_tracker.h"
#include "strategy/dubins.h"
#include "strategy/field.h"
#include "strategy/kick.h"
#include "strategy/motion.h"
#include "strategy/strategy.h"
#include "utils/geo/polygon.h"
#include "utils/mapper.h"
#include "utils/millis.h"
#include "utils/switch.h"

// enum class AttackerRSide { NONE, LEFT, RIGHT };
// enum class AttackerRStatus { NONE, TAKE_BALL, ROTATE_1, MOVE, ROTATE_2, KICK
// };

// static AttackerRSide r_side = AttackerRSide::NONE;
// static AttackerRStatus r_status = AttackerRStatus::NONE;

#define SPECIAL_HEIGHT config->strategy->attacker->special_height

static Polygon create_left_polygon() {
  return {{{0.0, 240.0 - SPECIAL_HEIGHT}, {0.0, 240.0}, ENEMY_GOAL_CENTER}};
}

static Polygon create_right_polygon() {
  return {{{182.0, 240.0 - SPECIAL_HEIGHT}, ENEMY_GOAL_CENTER, {182.0, 240.0}}};
}

static double weighted_dist(const Polygon& pol, const Vec& pos) {
  double ans = pol.dist(pos);
  if (!pol.inside(pos)) {
    ans *= -1;
  }
  return ans;
}

static Switch polygon_oscil{0, 2};

void Strategy::run_attacker(Robot& robot, Object& ball, Object& goal,
                            Field& field) {
  Polygon left_polygon = create_left_polygon();
  Polygon right_polygon = create_right_polygon();

  double left_polygon_dist = weighted_dist(left_polygon, robot.position);
  double right_polygon_dist = weighted_dist(right_polygon, robot.position);
  double polygon_dist = max(left_polygon_dist, right_polygon_dist);
  bool use_left_special = left_polygon_dist > right_polygon_dist;

  bool outside_special = polygon_oscil.compute(polygon_dist);
  bool inside_special = !outside_special;

  // ASSERTION
  if (!left_polygon.inside(robot.position) &&
      !right_polygon.inside(robot.position)) {
    assert(polygon_dist <= 0);
  }
  if (left_polygon.inside(robot.position) ||
      right_polygon.inside(robot.position)) {
    assert(polygon_dist >= 0);
  }

  Vec special_pos_left = {config->strategy->attacker->special_pos->x,
                          240.0 - config->strategy->attacker->special_pos->y};
  Vec special_pos_right = {182.0 - config->strategy->attacker->special_pos->x,
                           240.0 - config->strategy->attacker->special_pos->y};

  if (robot.emitter) {
    // Взяли мяч
    if (dubins_->was_active_last_tick()) {
      spdlog::info("KICK DUBINS");
      // Подъехали по окружности, используем удар оттуда
      dubins_->dubins_hit(robot, goal, field,
                          KickController::compute_power(robot.position), false);
    } else if (stabilize_capture(robot)) {
      // Только что захватили мяч: даём ему стабилизироваться (медленно
      // едем вперёд + рамп дриблера) до того как идти в special_zone или
      // в kick. Один раз на захват — стабилизатор сам отключается через
      // dribbling.param_r после first_time.
      spdlog::info("STABILIZE");
    } else {
      if (inside_special) {
        spdlog::info("SPECIAL");
        Vec target = use_left_special ? special_pos_left : special_pos_right;
        robot.dribbling = config->strategy->dribbling->value_r;
        drive_target(robot, target, 30);
        robot.rotation = 0;
      } else {
        spdlog::info("KICK GOAL");
        kick_->execute_to_goal(robot, goal, {});
      }
    }
  } else {
    // Не взяли мяч
    // r_status = AttackerRStatus::NONE;
    // r_side = AttackerRSide::NONE;
    bool recently_visible = ball_->recently_visible(millis(), 3000);
    Vec ball_pos = ball_->position();
    if (!recently_visible || ball_pos.y < config->strategy->attacker->border) {
      // Мяч у вратаря или мы его не видим
      Vec target;
      if (recently_visible && ball_pos.x < 91) {
        // Едем к левому краю
        target = {20, 120};
      } else {
        // Едем к правому краю
        target = {160, 120};
      }
      spdlog::info("DRIVE PASSIVE");
      drive_target(robot, target);
      robot.rotation = ball_->relative_angle(robot);
    } else {
      // Пробуем использовать dubins
      bool res = false;
      if (config->strategy->attacker->dubins_enabled) {
        res = dubins_->dubins_hit(robot, goal, field,
                                  KickController::compute_power(robot.position),
                                  false);
      }
      if (!res) {
        // Мяч близко к бортам, играем как обычно
        drive_ball(robot, ball_pos);
        // Компенсация скорости мяча: если мяч катится, добавляем его
        // скорость к команде, чтобы относительная скорость сближения
        // соответствовала тому, что drive_target насчитал для статичной цели.
        const auto& ff = *config->strategy->attacker->ff;
        if (ff.enabled && ball_->recently_visible(millis(), ff.stale_ms) &&
            ball_->velocity().len() >= ff.v_min) {
          robot.vel += ball_->velocity() * ff.gain;
        }
      }
    }
  }

  robot.vel = robot.vel.resize(min(robot.vel.len(), 120.0));
}