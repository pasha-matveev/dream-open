#pragma once

#include <opencv2/opencv.hpp>

double normalize_angle(double a);

class Vec {
   public:
    double x = 0, y = 0;
    Vec() = default;
    Vec(double, double);
    Vec(cv::Point2f);
    operator cv::Point();
    double len();
    double len2();
    double angle();

    friend Vec operator+(const Vec &, const Vec &);
    friend Vec operator-(const Vec &, const Vec &);
    friend double operator*(const Vec &, const Vec &);
    friend double operator%(const Vec &, const Vec &);
};