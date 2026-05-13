#include "config/strategy/keeper.h"

using namespace cfg;
using std::make_unique;

Keeper::~Keeper() = default;

Keeper::Keeper(const rapidjson::Value& doc) {
  projection_border = doc["projection_border"].GetDouble();

  line = make_unique<Line>();
  const rapidjson::Value& dline = doc["line"];
  line->padding = dline["padding"].GetDouble();
  line->y = dline["y"].GetDouble();
  line->ray_min_y = dline["ray_min_y"].GetDouble();

  ram = make_unique<Ram>();
  const rapidjson::Value& dram = doc["ram"];
  ram->enabled = dram["enabled"].GetBool();
  ram->safety_y = dram["safety_y"].GetDouble();
  ram->far_min_dist = dram["far_min_dist"].GetDouble();
  ram->blind_min_ms = dram["blind_min_ms"].GetInt64();
  ram->max_speed = dram["max_speed"].GetDouble();
  ram->min_speed = dram["min_speed"].GetDouble();
}
