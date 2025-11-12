#include "strategy/segment.h"

#include <spdlog/spdlog.h>

#include <cassert>

#include "robot.h"
#include "utils/geo/precision.h"

using namespace std;

Line::Line(const Vec& s, const Vec& f) {
  Vec dir = f - s;
  Vec normal = dir.turn_left();
  a = normal.x;
  b = normal.y;
  c = -a * s.x - b * s.y;
  double val = a * f.x + b * f.y + c;
}

// TODO: check parallel
Vec operator*(const Line& a, const Line& b) {
  double D = a.a * b.b - b.a * a.b;
  double x = (a.b * b.c - b.b * a.c) / D;
  double y = (a.c * b.a - b.c * a.a) / D;
  return {x, y};
}

Segment::Segment(const Vec& a_, const Vec& b_) : a(a_), b(b_) {}

bool Segment::is_proection(const Vec& p) const {
  auto v1 = (b - a) * (p - a);
  if (v1 < 0) return false;
  auto v2 = (a - b) * (p - b);
  if (v2 < 0) return false;
  return true;
}

bool Segment::correct_side(const Vec& p) const {
  return ((p - a) % (b - a)) > 0;
}

double Segment::normal_dist(const Vec& p) const {
  Vec l = b - a;
  double S = (b - p) % (a - p);
  double h = S / l.len();
  return h;
}

double Segment::dist(const Vec& p) const {
  if (is_proection(p)) {
    return abs(normal_dist(p));
  }
  return min((p - a).len(), (p - b).len());
}

void Segment::apply(Robot& robot) const {
  double d = normal_dist(robot.position);
  Vec ox = b - a;
  Vec oy = ox.turn_left();

  double vx = robot.vel.proection(ox);
  double vy = robot.vel.proection(oy);
  if (d <= 25) {
    vy = min(vy, d * 0.9);
  }

  Vec vvx = ox.resize(vx);
  Vec vvy = oy.resize(vy);

  robot.vel = vvx + vvy;
}

bool Segment::has_point(const Vec& p) const {
  double len = (p - a).len() + (p - b).len();
  return abs(len - (b - a).len()) <= EPS;
}

bool Segment::intersects_vel_deprecated(const Segment& vel) const {
  if (has_point(vel.a) || has_point(vel.b)) return true;
  if (vel.has_point(a) || vel.has_point(b)) return true;
  double v1 = (vel.a - a) % (b - a);
  double v2 = (b - a) % (vel.b - a);
  double v3 = (vel.b - vel.a) % (a - vel.a);
  double v4 = (b - vel.a) % (vel.b - vel.a);
  if (v1 == 0 || v2 == 0 || v3 == 0 || v4 == 0) return false;
  return v1 > 0 && v2 > 0 && v3 > 0 && v4 > 0;
}

Vec Segment::intersect_point(const Segment& vel) const {
  Line l1 = Line(a, b);
  Line l2 = Line(vel.a, vel.b);
  return l1 * l2;
}

Vec Segment::nearest_point(const Vec& p) const {
  if (has_point(p)) {
    return p;
  }
  if (is_proection(p)) {
    Vec dir = b - a;
    Vec normal = dir.turn_left();
    double val = (p - a) % dir;
    bool right = val > 0;
    if (!right) {
      normal *= -1;
    }
    normal = normal.resize(abs(normal_dist(p)));
    Vec q = p + normal;
    return q;
  } else {
    if ((p - a).len2() < (p - b).len2()) {
      return a;
    }
    return b;
  }
}

bool operator*(const Segment& p, const Segment& q) {
  if (p.has_point(q.a) || p.has_point(q.b)) return true;
  if (q.has_point(p.a) || q.has_point(p.b)) return true;
  double v1 = (q.a - p.a) % (p.b - p.a);
  double v2 = (q.b - p.a) % (p.b - p.a);
  double v3 = (p.a - q.a) % (q.b - q.a);
  double v4 = (p.b - q.a) % (q.b - q.a);
  bool c1 = (v1 < 0) != (v2 < 0);
  bool c2 = (v3 < 0) != (v4 < 0);
  return c1 && c2;
}
