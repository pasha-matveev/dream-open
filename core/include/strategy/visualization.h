#pragma once

#include <SFML/Graphics.hpp>
#include <memory>

#include "robot.h"
#include "strategy/field.h"
#include "tracking/object.h"
#include "utils/geo/polygon.h"

extern std::unique_ptr<sf::RenderWindow> sfml_window;

constexpr double K = 3;
constexpr int SFML_WIDTH = FIELD_WIDTH * K;
constexpr int SFML_HEIGHT = FIELD_HEIGHT * K;
constexpr sf::Color zone_color(255, 165, 0);

constexpr double cm_to_px(double x) { return x * K; }
constexpr double px_to_cm(double x) { return x / K; }

Vec toSFML(Vec r);
Vec toReal(Vec s);

constexpr double REAL_BALL_R = 2.1;
constexpr double REAL_ROBOT_R = 9;
static constexpr int BALL_R = cm_to_px(REAL_BALL_R);
static constexpr int ROBOT_R = cm_to_px(REAL_ROBOT_R);
static constexpr int MIN_BALL_X = 0 + BALL_R;
static constexpr int MAX_BALL_X = FIELD_WIDTH - BALL_R;
static constexpr int MIN_BALL_Y = 0 + BALL_R;
static constexpr int MAX_BALL_Y = FIELD_HEIGHT - BALL_R;

class Visualization {
 public:
  bool closed = false;
  Visualization();
  ~Visualization();
  void begin();
  bool running();
  void run(Robot& robot, Object& ball, Object& goal, const Field&, double dt);
  void draw_polygon(const Polygon&, const sf::Color& color);
};
