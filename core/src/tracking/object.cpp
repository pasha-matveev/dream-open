#include "tracking/object.h"

#include <vector>

#include "utils/config.h"

using namespace std;

Object::Object(const vector<int> &hsv_min, const vector<int> &hsv_max)
    : hsv_min(hsv_min), hsv_max(hsv_max) {}

Object::~Object() {};

float Object::get_pixels_dist() {
    Vec mirror_center = {config["tracking"]["center"]["x"].GetInt(),
                         config["tracking"]["center"]["y"].GetInt()};
    return (center - mirror_center).len();
}