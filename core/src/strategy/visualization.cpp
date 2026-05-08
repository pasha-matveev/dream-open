#include "strategy/visualization.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <optional>

#include "config/config.h"
#include "config/serial.h"
#include "config/visualization.h"
#include "strategy/strategy.h"

Vec toSFML(Vec r) {
  Vec v = {cm_to_px(r.x), cm_to_px(r.y)};
  return {v.x, SFML_HEIGHT - v.y};
}

Vec toReal(Vec s) {
  Vec v = {s.x, SFML_HEIGHT - s.y};
  return {px_to_cm(v.x), px_to_cm(v.y)};
}

std::unique_ptr<sf::RenderWindow> sfml_window;

Visualization::Visualization() {
  sfml_window = std::make_unique<sf::RenderWindow>(
      sf::VideoMode({(uint)SFML_WIDTH, (uint)SFML_HEIGHT}),
      config->visualization->window_name);
  sfml_window->setFramerateLimit(config->visualization->frames);
}

Visualization::~Visualization() {
  if (sfml_window) {
    if (sfml_window->isOpen()) {
      sfml_window->close();
    }
    sfml_window.reset();
  }
}

bool override_ball = false;
Vec ball_a{0, 0};

void Visualization::draw_polygon(const Polygon& field, const sf::Color& color) {
  sf::ConvexShape shape(field.points.size());
  for (int i = 0; i < field.points.size(); ++i) {
    shape.setPoint(i, toSFML(field.points[i]));
  }
  shape.setOutlineColor(color);
  shape.setOutlineThickness(2);
  shape.setFillColor(sf::Color::Transparent);
  if (!sfml_window) {
    return;
  }
  sfml_window->draw(shape);
}

void Visualization::begin() {
  if (!running()) return;
  sfml_window->clear({2, 179, 46});
}

bool Visualization::running() {
  if (closed) return false;
  if (!sfml_window || !sfml_window->isOpen()) {
    spdlog::info("Window is already closed, visualization not available");
    closed = true;
    return false;
  }
  while (const std::optional event = sfml_window->pollEvent()) {
    if (event->is<sf::Event::Closed>()) {
      spdlog::info("Closing visualization window");
      sfml_window->close();
      return false;
    }
  }
  return true;
}

void Visualization::run(Robot& robot, Object& ball, Object& goal,
                        const Field& field, double dt) {
  if (!running()) return;
  draw_polygon(field, sf::Color::White);
  // draw_polygon(left_attacker_r, zone_color);
  // draw_polygon(right_attacker_r, zone_color);

  if (config->visualization->interactive &&
      sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
    Vec mouse_position = sf::Mouse::getPosition(*sfml_window);
    override_ball = true;
    ball.field_position = toReal(mouse_position);
    ball_a = {0, 0};
    robot.emitter = false;
  }

  Vec robot_dir = {-1 * sin(robot.field_angle), cos(robot.field_angle)};

  // compute robot
  if (!config->serial->enabled) {
    if (robot.state == RobotState::PAUSE) {
      robot.state = RobotState::RUNNING;
    }
    // rotation: robot.rotation — это целевой относительный угол; делаем шаг к
    // нему, ограниченный rotation_limit (rad/s) с коэффициентом /3 для
    // совместимости со старым поведением.
    double max_rotation = robot.rotation_limit * dt / 3;
    robot.field_angle += clamp(robot.rotation, -1 * max_rotation, max_rotation);
    if (robot.emitter && override_ball) {
      ball.field_position = robot.position + robot_dir.resize(REAL_ROBOT_R);
    }

    // move
    robot.field_angle = normalize_angle(robot.field_angle);
    robot.position = robot.position + robot.actual_vel * dt;
  }

  if (override_ball) {
    ball.field_position += ball_a;
    if (ball.field_position.x < MIN_BALL_X) {
      ball_a.x = abs(ball_a.x);
      ball.field_position.x = MIN_BALL_X;
    } else if (ball.field_position.x > MAX_BALL_X) {
      ball_a.x = -abs(ball_a.x);
      ball.field_position.x = MAX_BALL_X;
    }
    if (ball.field_position.y < MIN_BALL_Y) {
      ball_a.y = abs(ball_a.y);
      ball.field_position.y = MIN_BALL_Y;
    } else if (ball.field_position.y > MAX_BALL_Y) {
      ball_a.y = -abs(ball_a.y);
      ball.field_position.y = MAX_BALL_Y;
    }
    ball_a = ball_a * 0.95;

    ball.visible = true;
    Vec v = ball.field_position - robot.position;
    Vec base = {-1 * sin(robot.field_angle), cos(robot.field_angle)};
    ball.relative_angle = normalize_angle(atan2f(base % v, base * v));
    ball.override_dist = v.len();

    if (ball.get_cm() <= REAL_ROBOT_R || robot.emitter) {
      robot.emitter = true;
      ball_a = {0, 0};
    } else {
      robot.emitter = false;
    }

    if (!config->serial->enabled && robot.kicker_force > 0 && robot.emitter) {
      robot.emitter = false;
      ball_a = robot_dir.resize(robot.kicker_force * 0.4);
      ball.field_position += ball_a;
    }
  }

  // draw points
  {
    Vec p1 = toSFML({52, 57});
    Vec p2 = toSFML({52, 240 - 57});
    Vec p3 = toSFML({182 - 52, 240 - 57});
    Vec p4 = toSFML({182 - 52, 57});
    auto s1 = sf::CircleShape(5);
    s1.setFillColor(sf::Color::Black);
    s1.setPosition(p1);
    sfml_window->draw(s1);

    auto s2 = sf::CircleShape(5);
    s2.setFillColor(sf::Color::Black);
    s2.setPosition(p2);
    sfml_window->draw(s2);

    auto s3 = sf::CircleShape(5);
    s3.setFillColor(sf::Color::Black);
    s3.setPosition(p3);
    sfml_window->draw(s3);

    auto s4 = sf::CircleShape(5);
    s4.setFillColor(sf::Color::Black);
    s4.setPosition(p4);
    sfml_window->draw(s4);
  }

  // draw robot
  {
    auto robot_shape = sf::CircleShape(ROBOT_R);
    robot_shape.setFillColor(sf::Color(0, 0, 0));
    Vec robot_point = toSFML(robot.position);
    robot_shape.setPosition(robot_point - Vec{ROBOT_R, ROBOT_R});
    sfml_window->draw(robot_shape);
    // draw hole
    auto hole_shape = sf::CircleShape(BALL_R);
    hole_shape.setFillColor({2, 179, 46});
    Vec move = {-1 * sin(robot.field_angle) * ROBOT_R,
                -1 * cos(robot.field_angle) * ROBOT_R};
    Vec hole_point = robot_point + move;
    hole_shape.setPosition(hole_point - Vec{BALL_R, BALL_R});
    sfml_window->draw(hole_shape);
  }

  // draw ball
  if (ball.visible) {
    auto ball_shape = sf::CircleShape(BALL_R);
    ball_shape.setFillColor(sf::Color(0, 0, 255));
    Vec point = toSFML(ball.field_position) - Vec{BALL_R, BALL_R};
    ball_shape.setPosition(point);
    sfml_window->draw(ball_shape);
  }

  sfml_window->display();
}
