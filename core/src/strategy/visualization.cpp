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
Vec ball_position;

const int BALL_R = cm_to_px(2.1);
const int ROBOT_R = cm_to_px(9);
float max_rotation = 30.0 / 60;

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
    ball_position = toReal(mouse_position);
  }

  if (override_ball) {
    ball.visible = true;
    Vec v = ball_position - robot.position;
    Vec base = {-1 * sin(robot.field_angle), cos(robot.field_angle)};
    ball.relative_angle = atan2f(base % v, base * v);
    ball.override_dist = v.len();
  } else {
    double ball_angle = robot.field_angle + ball.relative_angle;
    Vec d{-1 * sin(ball_angle) * ball.get_cm(),
          cos(ball_angle) * ball.get_cm()};
    ball_position = robot.position + d;
  }

  // compute robot
  if (!config["serial"]["enabled"].GetBool()) {
    if (robot.rotation < 0) {
      robot.field_angle += max(robot.rotation, -1 * max_rotation);
    } else {
      robot.field_angle += min(robot.rotation, max_rotation);
    }
    robot.field_angle = normalize_angle(robot.field_angle);
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
    Vec point = toSFML(ball_position) - Vec{BALL_R, BALL_R};
    ball_shape.setPosition(point);
    window.draw(ball_shape);
  }

  window.display();
}