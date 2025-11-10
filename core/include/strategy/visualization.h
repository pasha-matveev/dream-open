#pragma once

#include <SFML/Graphics.hpp>

#include "robot.h"
#include "strategy/field.h"
#include "tracking/object.h"
#include "utils/geo/polygon.h"

class Visualization {
 private:
  sf::RenderWindow window;

 public:
  bool closed = false;
  Visualization();
  void run(Robot& robot, Object& ball, Object& goal, const Field&);
  void draw_polygon(const Polygon&, const sf::Color& color);
};
