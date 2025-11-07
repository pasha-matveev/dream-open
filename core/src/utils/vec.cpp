#include "utils/vec.h"

#include <cmath>

using namespace std;

double normalize_angle(double a) {
  while (a <= -M_PI) {
    a += 2 * M_PI;
  }
  while (a > M_PI) {
    a -= 2 * M_PI;
  }
  return a;
}

double normalize_angle2(double a) {
  while (a < 0) {
    a += 2 * M_PI;
  }
  while (a >= 2 * M_PI) {
    a -= 2 * M_PI;
  }
  return a;
}

Vec::Vec(double x, double y) : x(x), y(y) {}
Vec::Vec(int x, int y) : x(x), y(y) {}
Vec::Vec(cv::Point2f p) : x(p.x), y(p.y) {}
Vec::Vec(sf::Vector2f v) : x(v.x), y(v.y) {}
Vec::Vec(sf::Vector2i v) : x(v.x), y(v.y) {}
Vec::Vec(double angle) : x(-1 * sin(angle)), y(cos(angle)) {}
Vec::operator cv::Point() const { return {(int)x, (int)y}; }
Vec::operator sf::Vector2f() const { return {(float)x, (float)y}; }
double Vec::len() const { return sqrt(x * x + y * y); }
double Vec::len2() const { return x * x + y * y; }
double Vec::raw_angle() const { return atan2(x, y); }
double Vec::field_angle() const {
  if (x == 0 && y == 0) {
    return 0;
  }
  return atan2(-x, y);
}
Vec Vec::operator*(double k) { return {x * k, y * k}; }
Vec& Vec::operator*=(double k) {
  x *= k;
  y *= k;
  return *this;
}
Vec& Vec::operator+=(const Vec& other) {
  x += other.x;
  y += other.y;
  return *this;
}

Vec operator+(const Vec& a, const Vec& b) { return {a.x + b.x, a.y + b.y}; }
Vec operator-(const Vec& a, const Vec& b) { return {a.x - b.x, a.y - b.y}; }
double operator*(const Vec& a, const Vec& b) { return a.x * b.x + a.y * b.y; }
double operator%(const Vec& a, const Vec& b) { return a.x * b.y - a.y * b.x; }
bool operator==(const Vec& a, const Vec& b) { return a.x == b.x && a.y == b.y; }

Vec Vec::resize(double target) const {
  if (len() == 0) {
    return {0, 0};
  }
  double k = target / len();
  return {x * k, y * k};
}
double Vec::proection(const Vec& v) const {
  double l = (*this * v) / v.len();
  return l;
}
Vec Vec::turn_left() const { return {-y, x}; }
Vec Vec::turn_right() const { return {y, -x}; }
Vec Vec::rotate(double angle) const {
  const double cs = cos(angle);
  const double sn = sin(angle);
  return {x * cs - y * sn, x * sn + y * cs};
}
