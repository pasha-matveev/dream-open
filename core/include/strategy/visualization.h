#include <SFML/Graphics.hpp>

class Visualization {
 private:
  sf::RenderWindow window;

 public:
  bool closed = false;
  Visualization();
  void run();
};