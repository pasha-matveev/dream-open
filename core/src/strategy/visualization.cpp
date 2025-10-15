#include "strategy/visualization.h"

#include <spdlog/spdlog.h>

#include "utils/config.h"

constexpr int REAL_WIDTH = 182;
constexpr int REAL_HEIGHT = 243;
constexpr double K = 3;
constexpr int SFML_WIDTH = REAL_WIDTH * K;
constexpr int SFML_HEIGHT = REAL_HEIGHT * K;

Visualization::Visualization() {
  window =
      sf::RenderWindow(sf::VideoMode({(uint)SFML_WIDTH, (uint)SFML_HEIGHT}),
                       config["visualization"]["window_name"].GetString());
  window.setFramerateLimit(config["visualization"]["frames"].GetInt());
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
Vec ball_position;
Vec ball_a{0, 0};

constexpr double REAL_BALL_R = 2.1;
constexpr double REAL_ROBOT_R = 9;
constexpr int BALL_R = cm_to_px(REAL_BALL_R);
constexpr int ROBOT_R = cm_to_px(REAL_ROBOT_R);

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
    ball_a = {0, 0};
  }

  Vec robot_dir = {-1 * sin(robot.field_angle), cos(robot.field_angle)};

  // compute robot
  if (!config["serial"]["enabled"].GetBool()) {
    // rotation
    float max_rotation = robot.rotation_limit / 60 / 3;
    if (robot.rotation < 0) {
      robot.field_angle += max(robot.rotation, -1 * max_rotation);
    } else {
      robot.field_angle += min(robot.rotation, max_rotation);
    }
    if (robot.emitter && override_ball) {
      Vec base{-1 * sin(robot.field_angle) * REAL_ROBOT_R,
               cos(robot.field_angle) * REAL_ROBOT_R};
      ball_position = robot.position + robot_dir.resize(REAL_ROBOT_R);
    }

    // move
    robot.field_angle = normalize_angle(robot.field_angle);
    Vec m = {
        (double)robot.speed * sin(robot.direction + robot.field_angle) * -1 /
            60,
        (double)robot.speed * cos(robot.direction + robot.field_angle) / 60};
    robot.position = robot.position + m;

    // kick

    if (robot.kicker_force > 0 && robot.emitter) {
      ball_a = robot_dir.resize(robot.kicker_force * 0.6);
    }
  }

  if (override_ball) {
    ball_position = ball_position + ball_a;
    if (ball_position.x <= 0 + BALL_R) {
      ball_a.x *= -1;
    } else if (ball_position.x >= REAL_WIDTH - BALL_R) {
      ball_a.x *= -1;
    }
    if (ball_position.y <= 0 + BALL_R) {
      ball_a.y *= -1;
    } else if (ball_position.y >= REAL_HEIGHT - BALL_R) {
      ball_a.y *= -1;
    }
    ball_a = ball_a * 0.96;

    ball.visible = true;
    Vec v = ball_position - robot.position;
    Vec base = {-1 * sin(robot.field_angle), cos(robot.field_angle)};
    ball.relative_angle = atan2f(base % v, base * v);
    ball.override_dist = v.len();

    if (ball.get_cm() <= 9) {
      robot.emitter = true;
      ball_a = {0, 0};
    } else {
      robot.emitter = false;
    }

  } else {
    double ball_angle = robot.field_angle + ball.relative_angle;
    Vec d{-1 * sin(ball_angle) * ball.get_cm(),
          cos(ball_angle) * ball.get_cm()};
    ball_position = robot.position + d;
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