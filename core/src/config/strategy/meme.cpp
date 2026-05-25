#include "config/strategy/meme.h"

#include <rapidjson/document.h>

using namespace cfg;

Meme::Meme(const rapidjson::Value& doc) {
  composition = doc["composition"].GetString();
  radius = doc["radius"].GetDouble();
  linear_speed = doc["linear_speed"].GetDouble();
  spin_speed = doc["spin_speed"].GetDouble();
}
