#include "config/strategy/attacker.h"

#include "utils/mapper.h"
#include "utils/switch.h"

using namespace cfg;
using std::make_unique;

Attacker::~Attacker() = default;

Attacker::Attacker(const rapidjson::Value& doc) {
  dubins_enabled = doc["dubins_enabled"].GetBool();
  special_height = doc["special_height"].GetDouble();

  fast_direct_enabled = doc["fast_direct_enabled"].GetBool();
  fast_direct = make_unique<Mapper>(doc["fast_direct"]);
  enemy_near_ball = make_unique<Switch>(doc["enemy_near_ball"]);
  enemy_min_y = doc["enemy_min_y"].GetDouble();
  enemy_latch_ms = doc["enemy_latch_ms"].GetInt64();
  fast_direct_dribbling = doc["fast_direct_dribbling"].GetInt();
  fast_direct_brake_safe = doc["fast_direct_brake_safe"].GetBool();

  special_pos = make_unique<SpecialPos>();
  special_pos->x = doc["special_pos"]["x"].GetDouble();
  special_pos->y = doc["special_pos"]["y"].GetDouble();

  ff = make_unique<Ff>();
  const rapidjson::Value& dff = doc["ff"];
  ff->enabled = dff["enabled"].GetBool();
  ff->v_min = dff["v_min"].GetDouble();
  ff->stale_ms = dff["stale_ms"].GetInt64();
  ff->gain = dff["gain"].GetDouble();

  spin_shot = make_unique<SpinShot>();
  const rapidjson::Value& ds = doc["spin_shot"];
  spin_shot->ready_dist = ds["ready_dist"].GetDouble();
  spin_shot->ready_angle = ds["ready_angle"].GetDouble();
  spin_shot->pre_shot_angle_offset = ds["pre_shot_angle_offset"].GetDouble();
  spin_shot->sweep_angle = ds["sweep_angle"].GetDouble();
  spin_shot->rotation_limit = ds["rotation_limit"].GetDouble();
  spin_shot->dribbling = make_unique<Mapper>(ds["dribbling"]);
  spin_shot->kicker_angle = ds["kicker_angle"].GetDouble();
  spin_shot->kicker_force = ds["kicker_force"].GetInt();
  spin_shot->spin_timeout_ms = ds["spin_timeout_ms"].GetInt();
  spin_shot->drive_max_speed = ds["drive_max_speed"].GetDouble();

  kurwa = make_unique<Kurwa>();
  const rapidjson::Value& dk = doc["kurwa"];
  kurwa->probability = dk["probability"].GetDouble();
  kurwa->pos_1 = make_unique<Vec>(dk["pos_1"]);
  kurwa->pos_2 = make_unique<Vec>(dk["pos_2"]);
  kurwa->target = make_unique<Vec>(dk["target"]);
  kurwa->turn_1_angle = dk["turn_1_angle"].GetDouble();
  kurwa->turn_2_angle = dk["turn_2_angle"].GetDouble();
  kurwa->ready_dist = dk["ready_dist"].GetDouble();
  kurwa->ready_angle = dk["ready_angle"].GetDouble();
  kurwa->drive_max_speed = dk["drive_max_speed"].GetDouble();
  kurwa->dribbling = dk["dribbling"].GetInt();

  icarus = make_unique<Icarus>();
  const rapidjson::Value& di = doc["icarus"];
  icarus->enabled = di["enabled"].GetBool();
  icarus->always_below_y = di["always_below_y"].GetDouble();
  icarus->curved_rotation = di["curved_rotation"].GetBool();
  icarus->recompute_angle_each_tick = di["recompute_angle_each_tick"].GetBool();
  icarus->target_left.x = di["target_left"]["x"].GetDouble();
  icarus->target_left.y = di["target_left"]["y"].GetDouble();
  icarus->target_right.x = di["target_right"]["x"].GetDouble();
  icarus->target_right.y = di["target_right"]["y"].GetDouble();
  icarus->dribbling_slowdown = make_unique<Mapper>(di["dribbling_slowdown"]);

  hobubu = make_unique<Hobubu>();
  const rapidjson::Value& dh = doc["hobubu"];
  hobubu->enabled = dh["enabled"].GetBool();
  hobubu->probability = dh["probability"].GetDouble();
  hobubu->wall_dist = dh["wall_dist"].GetDouble();
  hobubu->face_wall_angle = dh["face_wall_angle"].GetDouble();
  hobubu->drive_max_speed = dh["drive_max_speed"].GetDouble();
  hobubu->ready_dist = dh["ready_dist"].GetDouble();
  hobubu->ready_angle = dh["ready_angle"].GetDouble();
  hobubu->dribbling = dh["dribbling"].GetInt();
}
