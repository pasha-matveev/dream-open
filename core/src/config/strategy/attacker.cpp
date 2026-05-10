#include "config/strategy/attacker.h"

#include <optional>

using namespace cfg;
using std::make_unique;

Attacker::~Attacker() = default;

Attacker::Attacker(const rapidjson::Value& doc) {
  dubins_enabled = doc["dubins_enabled"].GetBool();
  special_height = doc["special_height"].GetDouble();

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
  int raw_dribbling = ds["dribbling_during_spin"].GetInt();
  spin_shot->dribbling_during_spin =
      raw_dribbling < 0 ? std::nullopt : std::optional<int>{raw_dribbling};
  spin_shot->kicker_ms = ds["kicker_ms"].GetInt();
  spin_shot->kicker_force = ds["kicker_force"].GetInt();
  spin_shot->spin_timeout_ms = ds["spin_timeout_ms"].GetInt();
}
