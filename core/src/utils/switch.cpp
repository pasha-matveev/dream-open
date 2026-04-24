#include "utils/switch.h"

void Switch::init(double middle, double deviation) {
  l = middle - deviation;
  r = middle + deviation;
}
Switch::Switch(double middle, double deviation) { init(middle, deviation); }
Switch::Switch(const rapidjson::Value& value) {
  double middle = value["middle"].GetDouble();
  double deviation = value["deviation"].GetDouble();
  init(middle, deviation);
}
bool Switch::compute(double val) {
  if (l <= val && val <= r) {
    return last_val;
  }
  if (val < l) {
    last_val = true;
  } else {
    last_val = false;
  }
  return last_val;
};