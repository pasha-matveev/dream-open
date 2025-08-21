#include "tracking/object.h"

#include <vector>

using namespace std;

Object::Object(vector<int> color_low, vector<int> color_high)
    : color_low(color_low), color_high(color_high) {}

Object::~Object() {};