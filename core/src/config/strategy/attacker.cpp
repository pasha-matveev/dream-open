#include "config/strategy/attacker.h"

#include "utils/mapper.h"
#include "utils/switch.h"

using namespace cfg;
using std::make_unique;

Attacker::~Attacker() = default;

Attacker::Attacker(const rapidjson::Value& doc) {
  dubins_enabled = doc["dubins_enabled"].GetBool();
  special_height = doc["special_height"].GetDouble();

  fast_direct = make_unique<Mapper>(doc["fast_direct"]);
  enemy_near_ball = make_unique<Switch>(doc["enemy_near_ball"]);
  enemy_min_y = doc["enemy_min_y"].GetDouble();
  enemy_latch_ms = doc["enemy_latch_ms"].GetInt64();
  fast_direct_dribbling = doc["fast_direct_dribbling"].GetInt();

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
  kurwa->pos_1 = make_unique<Kurwa::Pos>();
  kurwa->pos_1->x = dk["pos_1"]["x"].GetDouble();
  kurwa->pos_1->y = dk["pos_1"]["y"].GetDouble();
  kurwa->pos_2 = make_unique<Kurwa::Pos>();
  kurwa->pos_2->x = dk["pos_2"]["x"].GetDouble();
  kurwa->pos_2->y = dk["pos_2"]["y"].GetDouble();
  kurwa->turn_1_angle = dk["turn_1_angle"].GetDouble();
  kurwa->turn_2_angle = dk["turn_2_angle"].GetDouble();
  kurwa->ready_dist = dk["ready_dist"].GetDouble();
  kurwa->ready_angle = dk["ready_angle"].GetDouble();
  kurwa->drive_max_speed = dk["drive_max_speed"].GetDouble();
  kurwa->dribbling = dk["dribbling"].GetInt();
}
