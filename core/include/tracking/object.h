#pragma once

#include <vector>

#include "utils/vec.h"

class Object {
   public:
    std::vector<int> hsv_min;
    std::vector<int> hsv_max;
    Vec center;
    double angle;
    double dist;

    Object(const std::vector<int> &, const std::vector<int> &);
    virtual ~Object();
};