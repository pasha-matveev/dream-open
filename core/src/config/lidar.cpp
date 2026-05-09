#include "config/lidar.h"

using namespace cfg;
using std::make_unique;

Lidar::~Lidar() = default;
Lidar::Lidar(const rapidjson::Value& doc) {
  enabled = doc["enabled"].GetBool();
  path = doc["path"].GetString();

  calibration = make_unique<Calibration>();
  const rapidjson::Value& dcal = doc["calibration"];
  calibration->enabled = dcal["enabled"].GetBool();
  calibration->window = dcal["window"].GetInt();
  calibration->threshold = dcal["threshold"].GetInt();
  calibration->movement = dcal["movement"].GetDouble();
  calibration->angle = dcal["angle"].GetDouble();
  calibration->own_goal_timeout = dcal["own_goal_timeout"].GetInt();
}
