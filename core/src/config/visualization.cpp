#include "config/visualization.h"

using namespace cfg;

Visualization::~Visualization() = default;
Visualization::Visualization(const rapidjson::Value& doc) {
  enabled = doc["enabled"].GetBool();
  window_name = doc["window_name"].GetString();
  frames = doc["frames"].GetInt();
  interactive = doc["interactive"].GetBool();
}
