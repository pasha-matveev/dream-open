#include "config/strategy/attacker.h"

#include <spdlog/spdlog.h>

#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/dubins.h"
#include "strategy/ball_tracker.h"
#include "strategy/dubins.h"
#include "strategy/field.h"
#include "strategy/kick.h"
#include "strategy/motion.h"
#include "strategy/strategy.h"
#include "strategy/zones.h"
#include "utils/geo/polygon.h"
#include "utils/mapper.h"
#include "utils/millis.h"

#define SPECIAL_HEIGHT config->strategy->attacker->special_height

static Polygon create_left_polygon() {
  return {{{0.0, 243.0 - SPECIAL_HEIGHT}, {0.0, 243.0}, ENEMY_GOAL_CENTER}};
}

static Polygon create_right_polygon() {
  return {{{182.0, 243.0 - SPECIAL_HEIGHT}, ENEMY_GOAL_CENTER, {182.0, 243.0}}};
}

void Strategy::run_attacker(Robot& robot, Object& ball, Object& goal,
                            Field& field) {
  // Полигоны создаются один раз и переиспользуются между тиками — это нужно,
  // потому что они хранят встроенный Switch для hyst_inside.
  static Polygon left_polygon = create_left_polygon();
  static Polygon right_polygon = create_right_polygon();
  // Зона ответственности вратаря: keeper-зона + вырез под ворота. Вне неё
  // вратарь либо стоит на месте (PROJECTION или IDLE), либо защищает по
  // лучу не выезжая из своей зоны — мяч за нами не достанет, поэтому это
  // территория нападающего. Hysteresis ±2 см встроен в Polygon.
  static Polygon keeper_responsibility = make_keeper_responsibility();

  bool inside_left = left_polygon.hyst_inside(robot.position);
  bool inside_right = right_polygon.hyst_inside(robot.position);
  bool inside_special = inside_left || inside_right;
  // Выбор стороны: внутри одного из special — он и есть; иначе по x.
  bool use_left_special;
  if (inside_left) {
    use_left_special = true;
  } else if (inside_right) {
    use_left_special = false;
  } else {
    use_left_special = robot.position.x < 91.0;
  }

  Vec special_pos_left = {config->strategy->attacker->special_pos->x,
                          240.0 - config->strategy->attacker->special_pos->y};
  Vec special_pos_right = {182.0 - config->strategy->attacker->special_pos->x,
                           240.0 - config->strategy->attacker->special_pos->y};

  if (robot.emitter) {
    // Взяли мяч
    if (dubins_->was_active_last_tick() &&
        dubins_->is_aligned_for_kick(robot, goal)) {
      spdlog::info("KICK DUBINS");
      // Подъехали по окружности, используем удар оттуда. Гейт по углу
      // защищает от удара в свои ворота, если мяч был захвачен на ранней
      // фазе dubins, до того как корпус довернулся к воротам противника.
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
        // TODO: вынести в настройки
        drive_target(robot, target, 50);
        robot.rotation = 0;
      } else {
        spdlog::info("KICK GOAL");
        kick_->execute_to_goal(robot, goal, {});
      }
    }
  } else {
    // Не взяли мяч
    bool recently_visible = ball_->recently_visible(millis(), 3000);
    Vec ball_pos = ball_->position();
    bool ball_is_keepers =
        recently_visible && keeper_responsibility.hyst_inside(ball_pos);

    if (!recently_visible || ball_is_keepers) {
      // Мяч у вратаря или мы его не видим — отъезжаем на пассивную позицию.
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

  robot.vel = robot.vel.resize(std::min(robot.vel.len(), 120.0));
}
