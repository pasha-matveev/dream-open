#pragma once

#include <opencv2/opencv.hpp>

#include "SFML/Graphics.hpp"

double normalize_angle(double a);
double normalize_angle2(double a);

class Vec {
 public:
  double x = 0, y = 0;
  Vec() = default;
  Vec(double, double);
  Vec(int, int);
  Vec(cv::Point2f);
  Vec(sf::Vector2f);
  Vec(sf::Vector2i);
  operator cv::Point();
  operator sf::Vector2f();
  Vec operator*(double);
  Vec& operator*=(double k);
  Vec& operator+=(const Vec&);
  double len() const;
  double len2() const;
  double raw_angle() const;
  double field_angle() const;

  friend Vec operator+(const Vec&, const Vec&);
  friend Vec operator-(const Vec&, const Vec&);
  friend double operator*(const Vec&, const Vec&);
  friend double operator%(const Vec&, const Vec&);
  friend bool operator==(const Vec&, const Vec&);

  Vec resize(double) const;
  double proection(const Vec&) const;
  Vec turn_left() const;
};
