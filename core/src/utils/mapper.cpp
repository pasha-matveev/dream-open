#include "utils/mapper.h"

#include <algorithm>

using namespace std;

Mapper::Mapper(const rapidjson::Value& value) {
  param_l = value["param_l"].GetDouble();
  param_r = value["param_r"].GetDouble();
  value_l = value["value_l"].GetDouble();
  value_r = value["value_r"].GetDouble();
}

double Mapper::map(double val) {
  val = clamp(val, param_l, param_r);
  double k = (val - param_l) / (param_r - param_l);
  return value_l + (value_r - value_l) * k;
}