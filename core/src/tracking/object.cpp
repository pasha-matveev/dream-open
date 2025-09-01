#include "tracking/object.h"

#include <vector>

using namespace std;

Object::Object(const vector<int> &hsv_min, const vector<int> &hsv_max)
    : hsv_min(hsv_min), hsv_max(hsv_max) {}

Object::~Object() {};