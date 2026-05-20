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

  piter = make_unique<Piter>();
  const rapidjson::Value& dpiter = doc["piter"];
  piter->self_exclusion_radius = dpiter["self_exclusion_radius"].GetDouble();
  piter->markup_tolerance = dpiter["markup_tolerance"].GetDouble();

  field = make_unique<Field>();
  const rapidjson::Value& dfield = doc["field"];
  field->min_width = dfield["min_width"].GetDouble();
  field->max_width = dfield["max_width"].GetDouble();
  field->min_height = dfield["min_height"].GetDouble();
  field->max_height = dfield["max_height"].GetDouble();
}
