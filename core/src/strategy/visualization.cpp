#include "strategy/visualization.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <optional>

#include "strategy/strategy.h"
#include "utils/config.h"

static constexpr int REAL_WIDTH = 182;
static constexpr int REAL_HEIGHT = 243;
static constexpr double K = 3;
static constexpr int SFML_WIDTH = REAL_WIDTH * K;
static constexpr int SFML_HEIGHT = REAL_HEIGHT * K;
static const sf::Color zone_color(255, 165, 0);

Visualization::Visualization() {
  window =
      sf::RenderWindow(sf::VideoMode({(uint)SFML_WIDTH, (uint)SFML_HEIGHT}),
                       config.visualization.window_name);
  window.setFramerateLimit(config.visualization.frames);
}

static constexpr double cm_to_px(double x) { return x * K; }
static constexpr double px_to_cm(double x) { return x / K; }

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

static constexpr double REAL_BALL_R = 2.1;
static constexpr double REAL_ROBOT_R = 9;
static constexpr int BALL_R = cm_to_px(REAL_BALL_R);
static constexpr int ROBOT_R = cm_to_px(REAL_ROBOT_R);
static constexpr int MIN_BALL_X = 0 + BALL_R;
static constexpr int MAX_BALL_X = REAL_WIDTH - BALL_R;
static constexpr int MIN_BALL_Y = 0 + BALL_R;
static constexpr int MAX_BALL_Y = REAL_HEIGHT - BALL_R;

void Visualization::draw_polygon(const Polygon& field, const sf::Color& color) {
  sf::ConvexShape shape(field.points.size());
  for (int i = 0; i < field.points.size(); ++i) {
    shape.setPoint(i, toSFML(field.points[i]));
  }
  shape.setOutlineColor(color);
  shape.setOutlineThickness(5);
  shape.setFillColor(sf::Color::Transparent);
  window.draw(shape);
}

void Visualization::run(Robot& robot, Object& ball, Object& goal,
                        const Field& field) {
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
  window.clear({2, 179, 46});

  draw_polygon(field, sf::Color::White);
  draw_polygon(left_attacker_r, zone_color);
  draw_polygon(right_attacker_r, zone_color);

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
    if (robot.state == RobotState::PAUSE) {
      robot.state = RobotState::RUNNING;
    }
    // rotation
    double max_rotation = robot.rotation_limit / config.strategy.fps / 3;
    robot.field_angle += clamp(robot.rotation, -1 * max_rotation, max_rotation);
    if (robot.emitter && override_ball) {
      ball.field_position = robot.position + robot_dir.resize(REAL_ROBOT_R);
    }

    // move
    robot.field_angle = normalize_angle(robot.field_angle);
    Vec m = robot.vel.resize(robot.vel.len() / config.strategy.fps);

    robot.position = robot.position + m;
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
    ball.relative_angle = atan2f(base % v, base * v);
    ball.override_dist = v.len();

    if (ball.get_cm() <= REAL_ROBOT_R || robot.emitter) {
      robot.emitter = true;
      ball_a = {0, 0};
    } else {
      robot.emitter = false;
    }

    if (!config.serial.enabled && robot.kicker_force > 0 && robot.emitter) {
      robot.emitter = false;
      ball_a = robot_dir.resize(robot.kicker_force * 0.4);
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
    hole_shape.setFillColor({2, 179, 46});
    Vec move = {-1 * sin(robot.field_angle) * ROBOT_R,
                -1 * cos(robot.field_angle) * ROBOT_R};
    Vec hole_point = robot_point + move;
    hole_shape.setPosition(hole_point - Vec{BALL_R, BALL_R});
    window.draw(hole_shape);
  }

  if (ball.visible) {
    auto ball_shape = sf::CircleShape(BALL_R);
    ball_shape.setFillColor(sf::Color(0, 0, 255));
    Vec point = toSFML(ball.field_position) - Vec{BALL_R, BALL_R};
    ball_shape.setPosition(point);
    window.draw(ball_shape);
  }

  window.display();
}
