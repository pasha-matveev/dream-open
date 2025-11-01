#include "strategy/segment.h"

Segment::Segment(const Vec& a_, const Vec& b_) : a(a_), b(b_) {}

bool Segment::is_inside(Robot& robot) {
  Vec l = b - a;
  Vec f = robot.position - a;
  return (f % l) > 0;
}

double Segment::dist(const Vec& p) {
  Vec l = b - a;
  double S = (b - p) % (a - p);
  double h = S / l.len();
  return h;
}

bool Segment::proection(const Vec& p) {
  auto v1 = (b - a) * (p - a);
  if (v1 < 0) return false;
  auto v2 = (a - b) * (p - b);
  if (v2 < 0) return false;
  return true;
}

double Segment::total_dist(const Vec& p) {
  if (proection(p)) {
    return dist(p);
  }
  return min((p - a).len(), (p - b).len());
}

void Segment::apply(Robot& robot) {
  if (total_dist(robot.position) > 5) {
    return;
  }
  double d = dist(robot.position);
  Vec ox = b - a;
  Vec oy = ox.turn_left();

  double vx = robot.vel.proection(ox);
  double vy = robot.vel.proection(oy);
  if (is_inside(robot)) {
    assert(d >= 0);
  } else {
    assert(d <= 0);
  }
  if (d <= -1.7) {
    // робот снаружи, почти касается
    // значит, больше не можем удаляться от поля
    vy = min(vy, 0.0);
  } else if (d <= 0) {
    // робот снаружи, но не сильно
    // едем медленно
    vy = min(vy, 5.0);
  } else if (d <= 10) {
    // робот близко к границе
    vy = min(vy, 30.0);
  }

  Vec vvx = ox.resize(vx);
  Vec vvy = oy.resize(vy);

  robot.vel = vvx + vvy;
}