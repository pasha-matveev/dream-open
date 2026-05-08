#include "config/strategy/attacker.h"

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
}
