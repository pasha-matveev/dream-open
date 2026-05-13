#include "config/strategy/kickoff.h"

#include <cassert>

#include "field_dims.h"
#include "utils/mapper.h"

using namespace cfg;
using std::make_unique;

Kickoff::~Kickoff() = default;

Kickoff::Kickoff(const rapidjson::Value& doc) {
  dribbling = make_unique<Mapper>(doc["dribbling"]);
  capture_blind_timeout_ms = doc["capture_blind_timeout_ms"].GetInt64();
  capture_speed = doc["capture_speed"].GetDouble();

  const rapidjson::Value& dleft = doc["target_left"];
  target_left.x = dleft["x"].GetDouble();
  target_left.y = dleft["y"].GetDouble();

  const rapidjson::Value& dright = doc["target_right"];
  target_right.x = dright["x"].GetDouble();
  target_right.y = dright["y"].GetDouble();

  // Защита от опечатки в конфиге: target за пределами поля даёт молчаливо
  // неверный ricochet_field_angle (отражённая точка вне поля). Лучше
  // упасть на старте, чем в матче.
  assert(0 <= target_left.x && target_left.x <= field_dims::kWidth);
  assert(0 <= target_left.y && target_left.y <= field_dims::kHeight);
  assert(0 <= target_right.x && target_right.x <= field_dims::kWidth);
  assert(0 <= target_right.y && target_right.y <= field_dims::kHeight);

  kick_force = doc["kick_force"].GetInt();
  kick_followthrough_ms = doc["kick_followthrough_ms"].GetInt64();
  post_kickoff_delay_ms = doc["post_kickoff_delay_ms"].GetInt64();
}
