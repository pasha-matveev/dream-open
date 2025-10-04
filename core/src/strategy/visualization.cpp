#include "strategy/visualization.h"

#include <spdlog/spdlog.h>

#include "utils/config.h"

Visualization::Visualization() {
  window = sf::RenderWindow(sf::VideoMode({800, 800}),
                            config["visualization"]["window_name"].GetString());
  window.setFramerateLimit(config["visualization"]["frames"].GetInt());
}

void Visualization::run() {
  if (!window.isOpen()) {
    spdlog::info("Window is already closed, visualization not available");
    closed = true;
    return;
  }

  while (const std::optional event = window.pollEvent()) {
    if (event->is<sf::Event::Closed>()) {
      spdlog::info("Closing visualization window");
      window.close();
    }
  }
  window.clear(sf::Color::White);

  auto circle = sf::CircleShape(50);
  circle.setFillColor(sf::Color(0, 0, 0));
  window.draw(circle);

  window.display();
}