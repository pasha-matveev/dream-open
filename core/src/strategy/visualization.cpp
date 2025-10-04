#include "strategy/visualization.h"

#include <spdlog/spdlog.h>

#include "utils/config.h"

Visualization::Visualization() {
  window = sf::RenderWindow(sf::VideoMode({800, 800}),
                            config["visualization"]["window_name"].GetString());
  window.setFramerateLimit(config["visualization"]["frames"].GetInt());
}

double cm_to_px(double x) { return x / 200 * 800; }
double px_to_cm(double x) { return x / 800 * 200; }

Vec toSFML(Vec r) {
  Vec v = {cm_to_px(r.x), cm_to_px(r.y)};
  return {800 - v.y, v.x};
}

Vec toReal(Vec s) {
  Vec v = {s.y, 800 - s.x};
  return {px_to_cm(v.x), px_to_cm(v.y)};
}

bool override_ball = false;
Vec override_ball_position;

const int BALL_R = cm_to_px(2.1);

void Visualization::run(Robot &robot, Ball &ball) {
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

  if (config["visualization"]["interactive"].GetBool() &&
      sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
    Vec mouse_position = sf::Mouse::getPosition(window);
    cout << mouse_position.x << " " << mouse_position.y << endl;
    override_ball = true;
    override_ball_position = toReal(mouse_position);
  }

  if (override_ball) {
    ball.visible = true;
    ball.field_point = override_ball_position - robot.position;
  }

  if (ball.visible) {
    auto ball_shape = sf::CircleShape(BALL_R);
    ball_shape.setFillColor(sf::Color(0, 0, 255));
    Vec position = robot.ball_position(ball);
    Vec point = toSFML(position) - Vec{BALL_R, BALL_R};
    ball_shape.setPosition(point);
    window.draw(ball_shape);
  }

  window.display();
}