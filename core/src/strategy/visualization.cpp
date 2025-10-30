#include "strategy/visualization.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <optional>

#include "utils/config.h"

constexpr int REAL_WIDTH = 182;
constexpr int REAL_HEIGHT = 243;
constexpr double K = 3;
constexpr int SFML_WIDTH = REAL_WIDTH * K;
constexpr int SFML_HEIGHT = REAL_HEIGHT * K;

Visualization::Visualization() {
  window =
      sf::RenderWindow(sf::VideoMode({(uint)SFML_WIDTH, (uint)SFML_HEIGHT}),
                       config.visualization.window_name);
  window.setFramerateLimit(config.visualization.frames);
}

constexpr double cm_to_px(double x) { return x * K; }
constexpr double px_to_cm(double x) { return x / K; }

Vec toSFML(Vec r) {
  Vec v = {cm_to_px(r.x), cm_to_px(r.y)};
  return {v.x, SFML_HEIGHT - v.y};
}

Vec toReal(Vec s) {
  Vec v = {s.x, SFML_HEIGHT - s.y};
  return {px_to_cm(v.x), px_to_cm(v.y)};
}

bool override_ball = false;
Vec ball_a{0, 0};

constexpr double REAL_BALL_R = 2.1;
constexpr double REAL_ROBOT_R = 9;
constexpr int BALL_R = cm_to_px(REAL_BALL_R);
constexpr int ROBOT_R = cm_to_px(REAL_ROBOT_R);

void Visualization::run(Robot& robot, Ball& ball) {
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

  if (config.visualization.interactive &&
      sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
    Vec mouse_position = sf::Mouse::getPosition(window);
    override_ball = true;
    ball.field_position = toReal(mouse_position);
    ball_a = {0, 0};
    robot.emitter = false;
  }

  Vec robot_dir = {-1 * sin(robot.field_angle), cos(robot.field_angle)};

  // compute robot
  if (!config.serial.enabled) {
    // rotation
    float max_rotation = robot.rotation_limit / 60 / 3;
    if (robot.rotation < 0) {
      robot.field_angle += max(robot.rotation, -1 * max_rotation);
    } else {
      robot.field_angle += min(robot.rotation, max_rotation);
    }
    if (robot.emitter && override_ball) {
      ball.field_position = robot.position + robot_dir.resize(REAL_ROBOT_R);
    }

    // move
    robot.field_angle = normalize_angle(robot.field_angle);
    Vec m = {
        (double)robot.speed * sin(robot.direction + robot.field_angle) * -1 /
            60,
        (double)robot.speed * cos(robot.direction + robot.field_angle) / 60};
    robot.position = robot.position + m;
  }

  if (override_ball) {
    ball.field_position += ball_a;
    if (ball.field_position.x <= 0 + BALL_R) {
      ball_a.x *= -1;
    } else if (ball.field_position.x >= REAL_WIDTH - BALL_R) {
      ball_a.x *= -1;
    }
    if (ball.field_position.y <= 0 + BALL_R) {
      ball_a.y *= -1;
    } else if (ball.field_position.y >= REAL_HEIGHT - BALL_R) {
      ball_a.y *= -1;
    }
    ball_a = ball_a * 0.96;

    ball.visible = true;
    Vec v = ball.field_position - robot.position;
    Vec base = {-1 * sin(robot.field_angle), cos(robot.field_angle)};
    ball.relative_angle = atan2f(base % v, base * v);
    ball.override_dist = v.len();

    if (ball.get_cm() <= ROBOT_R) {
      robot.emitter = true;
      ball_a = {0, 0};
    } else {
      robot.emitter = false;
    }

    if (!config.serial.enabled && robot.kicker_force > 0 && robot.emitter) {
      robot.emitter = false;
      ball_a = robot_dir.resize(robot.kicker_force * 0.6);
      ball.field_position += ball_a;
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
    Vec point = toSFML(ball.field_position - Vec{BALL_R, BALL_R});
    ball_shape.setPosition(point);
    window.draw(ball_shape);
  }

  window.display();
}
