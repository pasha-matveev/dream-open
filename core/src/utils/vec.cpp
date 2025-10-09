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
Vec::operator cv::Point() { return {(int)x, (int)y}; }
Vec::operator sf::Vector2f() { return {(float)x, (float)y}; }
double Vec::len() { return hypot(x, y); }
double Vec::len2() { return x * x + y * y; }
double Vec::angle() { return atan2(x, y); }
Vec Vec::operator*(double k) { return {x * k, y * k}; }

Vec operator+(const Vec &a, const Vec &b) { return {a.x + b.x, a.y + b.y}; }
Vec operator-(const Vec &a, const Vec &b) { return {a.x - b.x, a.y - b.y}; }
double operator*(const Vec &a, const Vec &b) { return a.x * b.x + a.y * b.y; }
double operator%(const Vec &a, const Vec &b) { return a.x * b.y - a.y * b.x; }

Vec Vec::resize(double target) {
  double k = target / len();
  return {x * k, y * k};
}