#include "config/strategy/keeper.h"

using namespace cfg;
using std::make_unique;

Keeper::~Keeper() = default;

Keeper::Keeper(const rapidjson::Value& doc) {
  global_border = doc["global_border"].GetDouble();
  dubins_border = doc["dubins_border"].GetDouble();
  ram_enabled = doc["ram_enabled"].GetBool();

  line = make_unique<Line>();
  const rapidjson::Value& dline = doc["line"];
  line->padding = dline["padding"].GetDouble();
  line->y = dline["y"].GetDouble();
}
