#include "tracking/object.h"

#include <vector>

using namespace std;

Object::Object(vector<int> hsv_min, vector<int> hsv_max)
    : hsv_min(hsv_min), hsv_max(hsv_max) {}

Object::~Object() {};