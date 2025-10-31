#pragma once

#include <SFML/Graphics.hpp>

#include "robot.h"
#include "strategy/field.h"
#include "tracking/ball.h"

class Visualization {
 private:
  sf::RenderWindow window;

 public:
  bool closed = false;
  Visualization();
  void run(Robot& robot, Ball& ball, const Field&);
  void draw_field(const Field&);
};