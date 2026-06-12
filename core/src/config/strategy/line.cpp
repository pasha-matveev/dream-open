#include "config/strategy/line.h"

#include <rapidjson/document.h>

using namespace cfg;

Line::~Line() = default;

Line::Line(const rapidjson::Value& doc) {
  speed = doc["speed"].GetDouble();
  rotation_gain = doc["rotation_gain"].GetDouble();
  rotation_limit = doc["rotation_limit"].GetDouble();
  ring_radius_px = doc["ring_radius_px"].GetInt();
  v_threshold = doc["v_threshold"].GetInt();
  samples = doc["samples"].GetInt();
  min_arc_deg = doc["min_arc_deg"].GetDouble();
  max_arc_deg = doc["max_arc_deg"].GetDouble();
  max_turn_deg = doc["max_turn_deg"].GetDouble();
  lost_timeout_ms = doc["lost_timeout_ms"].GetInt();
}
