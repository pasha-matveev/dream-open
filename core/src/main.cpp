#include <spdlog/spdlog.h>

#include <chrono>
#include <thread>

#include "robot.h"
#include "strategy/field.h"
#include "strategy/strategy.h"
#include "strategy/visualization.h"
#include "tracking/object.h"
#include "utils/config.h"

using namespace std;

int main() {
  Field field({{12, 12}, {12, 231}, {170, 231}, {170, 12}});

  spdlog::info("Loading config...");
  load_config();
  spdlog::info("Config loaded");

  if (config.tracking.preview.enabled) {
    cv::namedWindow(config.tracking.preview.window_name, cv::WINDOW_AUTOSIZE);
  }

  Object ball(config.tracking.ball.hsv_min, config.tracking.ball.hsv_max,
              config.tracking.ball.setup, config.tracking.ball.min_area);
  Object goal(config.tracking.goal.type == "yellow"
                  ? config.tracking.goal.yellow.hsv_min
                  : config.tracking.goal.blue.hsv_min,
              config.tracking.goal.type == "yellow"
                  ? config.tracking.goal.yellow.hsv_max
                  : config.tracking.goal.blue.hsv_max,
              config.tracking.goal.setup, config.tracking.goal.min_area);
  Robot robot;
  spdlog::info("Initializing hardware...");
  robot.init_hardware(ball, goal);
  spdlog::info("Hardware ready");

  Strategy strategy;

  Visualization* visualization = nullptr;

  if (config.visualization.enabled) {
    visualization = new Visualization();
  }

  // int delay = 1000 / config.strategy.fps / 10;
  while (true) {
    if (config.serial.enabled) {
      robot.read_from_arduino();
    }
    // ... strategy ...
    strategy.run(robot, ball, goal, field);
    if (config.serial.enabled) {
      robot.write_to_arduino();
    }
    // visualisation
    if (config.visualization.enabled) {
      visualization->run(robot, ball, goal, field);
      if (visualization->closed) {
        break;
      }
    }
    // delay
    if (config.tracking.preview.enabled) {
      try {
        robot.camera->show_preview();
      } catch (...) {
      }
      if (cv::waitKey(5) == 27) {
        break;
      }
    } else {
      this_thread::sleep_for(chrono::milliseconds(5));
    }
  }
}
