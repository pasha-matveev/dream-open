#include "config/strategy/attacker.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <random>

#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/dubins.h"
#include "strategy/accel_drive.h"
#include "strategy/ball_tracker.h"
#include "strategy/dubins.h"
#include "strategy/field.h"
#include "strategy/kick.h"
#include "strategy/kurwa.h"
#include "strategy/motion.h"
#include "strategy/obstacles.h"
#include "strategy/spin_pipeline.h"
#include "strategy/spin_shot.h"
#include "strategy/strategy.h"
#include "strategy/zones.h"
#include "utils/geo/polygon.h"
#include "utils/geo/vec.h"
#include "utils/mapper.h"
#include "utils/millis.h"
#include "utils/switch.h"

#define SPECIAL_HEIGHT config->strategy->attacker->special_height

static Polygon create_left_polygon() {
  return {{{0.0, 243.0 - SPECIAL_HEIGHT}, {0.0, 243.0}, ENEMY_GOAL_CENTER}};
}

static Polygon create_right_polygon() {
  return {{{182.0, 243.0 - SPECIAL_HEIGHT}, ENEMY_GOAL_CENTER, {182.0, 243.0}}};
}

namespace {
// Диспетчер атак: какой контроллер сейчас активен. NONE = ни spin, ни kurwa.
// Бросок монеты делается один раз на переходе NONE → SPIN/KURWA, держится
// до завершения атаки или потери мяча.
enum class AttackKind { NONE, SPIN, KURWA };
struct AttackState {
  AttackKind kind = AttackKind::NONE;
  int prev_first_time = -100000;
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
  // Состояние диспетчера атак. Той же природы, что и static-полигоны
  // выше — нужно между тиками, не нужно никому за пределами run_attacker.
  static AttackState attack;
  // Время последней детекции "враг у мяча"; -1 = не было. Латч переживает
  // пропажи цели лидаром между кадрами (см. enemy_latch_ms).
  static long long enemy_near_since = -1;

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

  // Сброс диспетчера на потере мяча / новом захвате.
  bool fresh_capture =
      robot.emitter && (robot.first_time != attack.prev_first_time);
  if (!robot.emitter || fresh_capture) {
    attack.kind = AttackKind::NONE;
  }
  attack.prev_first_time = robot.first_time;

  // Атака считается "engaged" с момента, как мы выбрали тип. С этой точки
  // ведём до завершения независимо от inside_special.
  bool engaged = attack.kind != AttackKind::NONE;

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
    } else if (stabilize_capture(robot,
                                 {.dribbling = *config->strategy->dribbling})) {
      // Только что захватили мяч: даём ему стабилизироваться (медленно
      // едем вперёд + разгон дриблера) до того как идти в special_zone или
      // в kick. Один раз на захват — стабилизатор сам отключается через
      // dribbling.param_r после first_time.
      spdlog::info("STABILIZE");
    } else {
      if (inside_special || engaged) {
        if (attack.kind == AttackKind::NONE) {
          // Бросок монеты — один раз на активацию. Seed от random_device;
          // если понадобится детерминированность в тестах, можно временно
          // подменить на rng.seed(...).
          static thread_local std::mt19937 rng(std::random_device{}());
          std::uniform_real_distribution<double> dist(0.0, 1.0);
          double r = dist(rng);
          double p = config->strategy->attacker->kurwa->probability;
          attack.kind = (r < p) ? AttackKind::KURWA : AttackKind::SPIN;
          spdlog::info("ATTACK START: {}",
                       attack.kind == AttackKind::KURWA ? "kurwa" : "spin");
        }

        bool done = false;
        if (attack.kind == AttackKind::SPIN) {
          done = spin_pipeline_->execute(robot, use_left_special,
                                         *config->strategy->attacker);
        } else {
          done = kurwa_->execute(robot, goal, use_left_special,
                                 *config->strategy->attacker);
        }
        if (done) attack.kind = AttackKind::NONE;
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
        target = {45, 105};
      } else {
        // Едем к правому краю
        target = {137, 105};
      }
      spdlog::info("DRIVE PASSIVE");
      drive_target(robot, target);
      robot.rotation = ball_->relative_angle(robot);
    } else {
      // Враг рядом с мячом? Лидаром ищем препятствие, ближайшее к мячу;
      // зону вратаря отбрасываем по y — лидар не отличает своих от чужих.
      auto enemy = nearest_obstacle(robot, ball_pos,
                                    config->strategy->attacker->enemy_min_y);
      bool detected_now = false;
      if (enemy) {
        double d = (*enemy - ball_pos).len();
        detected_now = config->strategy->attacker->enemy_near_ball->compute(d);
      }
      if (detected_now) enemy_near_since = millis();
      bool enemy_near_ball =
          enemy_near_since >= 0 &&
          millis() - enemy_near_since <=
              config->strategy->attacker->enemy_latch_ms;

      if (enemy_near_ball) {
        // Враг захватил мяч и убегает — dubins бесполезен. Едем напрямую и
        // быстрее (fast_direct), дриблер сразу на максимум, чтобы выхватить
        // мяч на подъезде.
        drive_target(robot, ball_pos, 120, 0, true,
                     config->strategy->attacker->fast_direct.get());
        robot.dribbling = config->strategy->attacker->fast_direct_dribbling;
        robot.rotation_limit = 40;
      } else {
        // Пробуем использовать dubins
        bool res = false;
        if (config->strategy->attacker->dubins_enabled) {
          res = dubins_->dubins_hit(
              robot, goal, field,
              KickController::compute_power(robot.position), false);
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
  }

  robot.vel = robot.vel.resize(std::min(robot.vel.len(), 120.0));
}
