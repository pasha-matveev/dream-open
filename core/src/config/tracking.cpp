#include "config/tracking.h"

#include "config/to_vector.h"

using namespace cfg;
using std::make_unique;

Tracking::~Tracking() = default;
Tracking::Tracking(const rapidjson::Value& doc) {
  enabled = doc["enabled"].GetBool();
  preview_enabled = doc["preview_enabled"].GetBool();
  camera_id = doc["camera_id"].GetInt();
  retries = doc["retries"].GetInt();
  fps = doc["fps"].GetInt();
  width = doc["width"].GetInt();
  height = doc["height"].GetInt();
  brightness = doc["brightness"].GetDouble();
  radius = doc["radius"].GetInt();
  disabled_radius = doc["disabled_radius"].GetInt();

  center = make_unique<Center>();
  const rapidjson::Value& dcenter = doc["center"];
  center->x = dcenter["x"].GetInt();
  center->y = dcenter["y"].GetInt();

  formula = make_unique<Formula>();
  const rapidjson::Value& dformula = doc["formula"];
  formula->a = dformula["a"].GetDouble();
  formula->b = dformula["b"].GetDouble();
  formula->c = dformula["c"].GetDouble();

  ball = make_unique<Ball>();
  const rapidjson::Value& dball = doc["ball"];
  ball->setup = dball["setup"].GetBool();
  ball->min_area = dball["min_area"].GetInt();
  ball->hsv_min = to_vector<int>(dball["hsv_min"]);
  ball->hsv_max = to_vector<int>(dball["hsv_max"]);

  goal = make_unique<Goal>();
  const rapidjson::Value& dgoal = doc["goal"];
  goal->type = dgoal["type"].GetString();
  goal->setup = dgoal["setup"].GetBool();
  goal->min_area = dgoal["min_area"].GetInt();

  goal->yellow = make_unique<CameraObject>();
  goal->yellow->hsv_min = to_vector<int>(dgoal["yellow"]["hsv_min"]);
  goal->yellow->hsv_max = to_vector<int>(dgoal["yellow"]["hsv_max"]);

  goal->blue = make_unique<CameraObject>();
  goal->blue->hsv_min = to_vector<int>(dgoal["blue"]["hsv_min"]);
  goal->blue->hsv_max = to_vector<int>(dgoal["blue"]["hsv_max"]);
}