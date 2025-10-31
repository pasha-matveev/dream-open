#include "strategy/segment.h"

Segment::Segment(const Vec& a_, const Vec& b_) : a(a_), b(b_) {}

bool Segment::is_inside(Robot& robot) {
  Vec l = b - a;
  Vec f = robot.position - a;
  return (f % l) > 0;
}

double Segment::dist(Vec& p) {
  Vec l = b - a;
  double S = abs((a - p) % (b - p));
  double h = S / l.len();
  return h;
}

void Segment::apply(Robot& robot) {
  double d = dist(robot.position);
  Vec ox = b - a;
  Vec oy = ox.turn_left();

  double vx = robot.vel.proection(ox);
  double vy = robot.vel.proection(oy);
  if (d <= 2) {
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