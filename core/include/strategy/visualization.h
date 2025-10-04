#include <SFML/Graphics.hpp>

#include "robot.h"
#include "tracking/ball.h"

class Visualization {
 private:
  sf::RenderWindow window;

 public:
  bool closed = false;
  Visualization();
  void run(Robot &robot, Ball &ball);
};