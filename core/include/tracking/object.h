#pragma once
#include <vector>

#include "vec.h"

class Object {
   public:
    std::vector<int> color_low;
    std::vector<int> color_high;
    Vec center;
    double angle;
    double dist;

    Object(std::vector<int> color_low, std::vector<int> color_high);
    virtual ~Object();
};