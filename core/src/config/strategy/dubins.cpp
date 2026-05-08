#include "config/strategy/dubins.h"

#include "utils/mapper.h"
#include "utils/switch.h"

using namespace cfg;
using std::make_unique;

Dubins::~Dubins() = default;

Dubins::Dubins(const rapidjson::Value& doc) {
  bonus = doc["bonus"].GetDouble();
  separate = doc["separate"].GetDouble();
  radius = doc["radius"].GetDouble();
  deep_inside = doc["deep_inside"].GetDouble();
  camera_target_dist = doc["camera_target_dist"].GetDouble();
  kick_precision = make_unique<Switch>(doc["kick_precision"]);
  aim_bonus = doc["aim_bonus"].GetDouble();
  kick_angle_tolerance = doc["kick_angle_tolerance"].GetDouble();
  speed = make_unique<Mapper>(doc["speed"]);
}
