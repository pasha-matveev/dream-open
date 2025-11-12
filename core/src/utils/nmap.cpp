#include "utils/nmap.h"

#include <algorithm>

using namespace std;

double nmap(double val, double l, double r, double L, double R) {
  val = clamp(val, l, r);
  double k = (val - l) / (r - l);
  return L + (R - L) * k;
}