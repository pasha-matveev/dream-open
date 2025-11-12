#include "utils/geo/circle.h"

#include <cmath>
#include <iostream>

using namespace std;

bool Circle::inside(const Vec& p) const { return (p - center).len2() <= r * r; }

double Circle::dist(const Vec& p) const { return (p - center).len() - r; }

double Circle::tangent_angle(const Vec& p) const {
  Vec l = center - p;
  double si = clamp(r / l.len(), -1.0, 1.0);
  double ang = asin(si);
  return ang;
}

double Circle::tangent_len(const Vec& p) const {
  assert(!inside(p));
  Vec l = center - p;
  double len = sqrt(l.len2() - r * r);
  assert(len == len);
  return len;
}

Vec Circle::tangent_left(const Vec& p) const {
  assert(!inside(p));
  Vec d = (center - p).rotate(tangent_angle(p)).resize(tangent_len(p));
  assert(d.x == d.x);
  Vec t = p + d;
  return t;
}

Vec Circle::tangent_right(const Vec& p) const {
  assert(!inside(p));
  Vec d = (center - p).rotate(-tangent_angle(p)).resize(tangent_len(p));
  assert(d.x == d.x);
  Vec t = p + d;
  return t;
}