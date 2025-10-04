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
  return {v.x, 800 - v.y};
}

Vec toReal(Vec s) {
  Vec v = {s.x, 800 - s.y};
  return {px_to_cm(v.x), px_to_cm(v.y)};
}

bool override_ball = false;
Vec override_ball_position;

const int BALL_R = cm_to_px(2.1);
const int ROBOT_R = cm_to_px(9);

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
    override_ball = true;
    override_ball_position = toReal(mouse_position);
  }

  if (override_ball) {
    ball.visible = true;
    ball.field_point = override_ball_position - robot.position;
  }

  // compute robot
  if (!config["serial"]["enabled"].GetBool()) {
    robot.field_angle += robot.rotation;
    robot.field_angle = normalize_angle(robot.field_angle);
    if (override_ball) {
      Vec base = {-1 * sin(robot.field_angle), cos(robot.field_angle)};
      Vec v = override_ball_position - robot.position;
      ball.relative_angle = atan2f(base % v, base * v);
    }
  }

  // draw robot
  {
    auto robot_shape = sf::CircleShape(ROBOT_R);
    robot_shape.setFillColor(sf::Color(0, 0, 0));
    Vec robot_point = toSFML(robot.position);
    robot_shape.setPosition(robot_point - Vec{ROBOT_R, ROBOT_R});
    window.draw(robot_shape);
    // draw hole
    auto hole_shape = sf::CircleShape(BALL_R);
    hole_shape.setFillColor(sf::Color(255, 255, 255));
    Vec move = {-1 * sin(robot.field_angle) * ROBOT_R,
                -1 * cos(robot.field_angle) * ROBOT_R};
    Vec hole_point = robot_point + move;
    hole_shape.setPosition(hole_point - Vec{BALL_R, BALL_R});
    window.draw(hole_shape);
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