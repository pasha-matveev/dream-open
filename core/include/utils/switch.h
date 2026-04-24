#pragma once

#include <rapidjson/rapidjson.h>

// Оператор "<" с пороговым значением
class Switch {
 private:
  double l, r;
  bool last_val = false;
  void init(double middle, double deviation);

 public:
  Switch() = default;
  Switch(double middle, double deviation);
  Switch(const rapidjson::Value& value);
  bool compute(double val);
};