#include "config/strategy/attacker.h"

#include <spdlog/spdlog.h>

#include <cmath>

#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/dubins.h"
#include "strategy/ball_tracker.h"
#include "strategy/dubins.h"
#include "strategy/field.h"
#include "strategy/kick.h"
#include "strategy/motion.h"
#include "strategy/spin_shot.h"
#include "strategy/strategy.h"
#include "strategy/zones.h"
#include "utils/geo/polygon.h"
#include "utils/geo/vec.h"
#include "utils/mapper.h"
#include "utils/millis.h"

#define SPECIAL_HEIGHT config->strategy->attacker->special_height

static Polygon create_left_polygon() {
  return {{{0.0, 243.0 - SPECIAL_HEIGHT}, {0.0, 243.0}, ENEMY_GOAL_CENTER}};
}

static Polygon create_right_polygon() {
  return {{{182.0, 243.0 - SPECIAL_HEIGHT}, ENEMY_GOAL_CENTER, {182.0, 243.0}}};
}

namespace {
// Состояние spin-shot пайплайна нападающего. Живёт как function-static в
// run_attacker (как и static-полигоны выше) — потому что это срез логики
// именно этой ветки, никому больше не нужен.
enum class SpinPhase { HIDE_TURN, DRIVE, PRE_SHOT_TURN, SPIN };
struct SpinPipelineState {
  SpinPhase phase = SpinPhase::HIDE_TURN;
  int prev_first_time = -100000;
  // Сторона: синк с use_left_special пока робот внутри special, после
  // выхода (в т.ч. в DRIVE) значение замораживается на последнем live.
  // Это нужно потому что target_pos лежит за пределами special_zone, и
  // во время DRIVE робот пересекает границу — без заморозки use_left
  // мог бы флипнуться при пересечении центра поля.
  bool use_left = false;
};
}  // namespace

void Strategy::run_attacker(Robot& robot, Object& ball, Object& goal,
                            Field& field) {
  // Полигоны создаются один раз и переиспользуются между тиками — это нужно,
  // потому что они хранят встроенный Switch для hyst_inside.
  static Polygon left_polygon = create_left_polygon();
  static Polygon right_polygon = create_right_polygon();
  // Зона ответственности вратаря: keeper-зона + вырез под ворота
  static Polygon keeper_responsibility = make_keeper_responsibility();
  // Состояние spin-shot пайплайна. Той же природы, что и static-полигоны
  // выше — нужно между тиками, не нужно никому за пределами run_attacker.
  static SpinPipelineState spin;

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

  static Vec special_pos_left = {
      config->strategy->attacker->special_pos->x,
      240.0 - config->strategy->attacker->special_pos->y};
  static Vec special_pos_right = {
      182.0 - config->strategy->attacker->special_pos->x,
      240.0 - config->strategy->attacker->special_pos->y};

  // Сброс spin-pipeline
  bool fresh_capture =
      robot.emitter && (robot.first_time != spin.prev_first_time);
  if (!robot.emitter || fresh_capture) {
    spin.phase = SpinPhase::HIDE_TURN;
  }
  spin.prev_first_time = robot.first_time;

  // Пайплайн "engaged" с момента, как HIDE_TURN отработал и стартовал DRIVE.
  // С этой точки ведём до завершения SPIN независимо от inside_special.
  bool spin_engaged = spin.phase != SpinPhase::HIDE_TURN;

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
      // едем вперёд + разгон дриблера) до того как идти в special_zone или
      // в kick. Один раз на захват — стабилизатор сам отключается через
      // dribbling.param_r после first_time.
      spdlog::info("STABILIZE");
    } else {
      if (inside_special || spin_engaged) {
        const auto& spin_cfg = *config->strategy->attacker->spin_shot;
        if (inside_special) {
          spin.use_left = use_left_special;
        }
        Vec target_pos = spin.use_left ? special_pos_left : special_pos_right;
        robot.dribbling = config->strategy->dribbling->value_r;

        // wall_angle: ориентация корпуса лицом к ближайшей вертикальной
        // стенке (HIDE_TURN/DRIVE)
        double wall_angle = spin.use_left ? +M_PI / 2 : -M_PI / 2;
        // pre_shot_angle: базис M_PI (лицом к нашим воротам, "вниз")
        // плюс отклонение в сторону ближайшей стенки. На левой стороне
        // -offset уходит в сторону +π/2 (левая стенка), на правой
        // +offset уходит в сторону -π/2 через wrap (правая стенка).
        double pre_shot_delta =
            (spin.use_left ? -1 : 1) * spin_cfg.pre_shot_angle_offset;
        double pre_shot_angle = M_PI + pre_shot_delta;

        switch (spin.phase) {
          case SpinPhase::HIDE_TURN: {
            spdlog::info("SPIN HIDE_TURN");
            double delta = normalize_angle(wall_angle - robot.field_angle);
            robot.vel = {0, 0};
            robot.rotation = delta;
            if (std::abs(delta) < spin_cfg.ready_angle) {
              spin.phase = SpinPhase::DRIVE;
            }
            break;
          }
          case SpinPhase::DRIVE: {
            spdlog::info("SPIN DRIVE");
            drive_target(robot, target_pos, 30);
            // drive_target сам ставит rotation, поэтому override после.
            robot.rotation = normalize_angle(wall_angle - robot.field_angle);
            if ((target_pos - robot.position).len() < spin_cfg.ready_dist)
              spin.phase = SpinPhase::PRE_SHOT_TURN;
            break;
          }
          case SpinPhase::PRE_SHOT_TURN: {
            spdlog::info("SPIN PRE_SHOT_TURN");
            double delta = normalize_angle(pre_shot_angle - robot.field_angle);
            robot.vel = {0, 0};
            robot.rotation = delta;
            if (std::abs(delta) < spin_cfg.ready_angle)
              spin.phase = SpinPhase::SPIN;
            break;
          }
          case SpinPhase::SPIN: {
            spdlog::info("SPIN");

            SpinShotParams p;
            p.direction = spin.use_left ? -1 : +1;
            p.sweep_angle = spin_cfg.sweep_angle;
            p.rotation_limit = spin_cfg.rotation_limit;
            p.dribbling = spin_cfg.dribbling_during_spin;
            p.kicker_angle = spin_cfg.kicker_angle;
            p.kicker_force = spin_cfg.kicker_force;
            p.spin_timeout_ms = spin_cfg.spin_timeout_ms;

            bool done = spin_shot_->execute(robot, p);
            if (done) spin.phase = SpinPhase::HIDE_TURN;
            break;
          }
        }
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
