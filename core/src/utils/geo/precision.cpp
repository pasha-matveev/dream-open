#include "utils/geo/precision.h"

#include <cmath>

bool eq(double a, double b) { return abs(a - b) <= EPS; }