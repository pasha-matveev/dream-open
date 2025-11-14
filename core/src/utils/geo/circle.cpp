#include "utils/geo/circle.h"

#include <cmath>
#include <iostream>

#include "strategy/visualization.h"

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

void Circle::draw() const {
  if (!sfml_window || !sfml_window->isOpen()) {
    return;
  }

  auto sfml_r = cm_to_px(r);
  auto shape = sf::CircleShape(sfml_r);
  shape.setPosition(toSFML(center) - Vec{sfml_r, sfml_r});
  shape.setOutlineColor(sf::Color(255, 0, 0));
  shape.setOutlineThickness(2);
  shape.setFillColor(sf::Color::Transparent);
  sfml_window->draw(shape);
}

double Circle::len() const { return M_PI * 2 * r; }

double Circle::path_len(const Vec& p, const Vec& q, bool to_left) const {
  Vec a = (p - center);
  Vec b = (q - center);
  double len_angle;
  if (to_left) {
    len_angle = normalize_angle2(b.field_angle() - a.field_angle());
  } else {
    len_angle = normalize_angle2(a.field_angle() - b.field_angle());
  }
  return len() * len_angle / (2 * M_PI);
}
