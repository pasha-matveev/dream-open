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
}
