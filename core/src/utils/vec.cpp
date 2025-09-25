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

Vec::Vec(double x, double y) : x(x), y(y) {}
Vec::Vec(int x, int y) : x(x), y(y) {}
Vec::Vec(cv::Point2f p) : x(p.x), y(p.y) {}
Vec::operator cv::Point() { return {(int)x, (int)y}; }
double Vec::len() { return hypot(x, y); }
double Vec::len2() { return x * x + y * y; }
double Vec::angle() { return atan2(x, y); }

Vec operator+(const Vec &a, const Vec &b) { return {a.x + b.x, a.y + b.y}; }
Vec operator-(const Vec &a, const Vec &b) { return {a.x - b.x, a.y - b.y}; }
double operator*(const Vec &a, const Vec &b) { return a.x * b.x + a.y + b.y; }
double operator%(const Vec &a, const Vec &b) { return a.x * b.y - a.y * b.x; }