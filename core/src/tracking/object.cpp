#include "tracking/object.h"

#include <vector>

#include "utils/config.h"

using namespace std;

Object::Object(const vector<int> &hsv_min, const vector<int> &hsv_max) {
  h_min = hsv_min[0];
  s_min = hsv_min[1];
  v_min = hsv_min[2];
  h_max = hsv_max[0];
  s_max = hsv_max[1];
  v_max = hsv_max[2];
}

Object::~Object() {};

float Object::get_pixels_dist() {
  int radius = config["tracking"]["radius"].GetInt();
  Vec mirror_center = {radius, radius};
  return (center - mirror_center).len();
  // return (center - mirror_center).len() * 972.0 /
  //        (float)config["tracking"]["width"].GetInt();
}