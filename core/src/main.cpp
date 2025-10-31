#include <spdlog/spdlog.h>

#include <chrono>
#include <thread>

#include "robot.h"
#include "strategy/field.h"
#include "strategy/strategy.h"
#include "strategy/visualization.h"
#include "tracking/ball.h"
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

  Ball ball(config.tracking.ball.hsv_min, config.tracking.ball.hsv_max,
            config.tracking.preview.setup);

  Robot robot;
  spdlog::info("Initializing hardware...");
  robot.init_hardware(ball);
  spdlog::info("Hardware ready");

  Strategy strategy;

  Visualization* visualization = nullptr;

  if (config.visualization.enabled) {
    visualization = new Visualization();
  }

  int delay = 1000 / config.strategy.fps / 10;
  while (true) {
    if (config.serial.enabled) {
      robot.read_from_arduino();
    }
    // ... strategy ...
    strategy.run(robot, ball, field);
    if (config.serial.enabled) {
      robot.write_to_arduino();
    }
    // visualisation
    if (config.visualization.enabled) {
      visualization->run(robot, ball, field);
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
      if (cv::waitKey(10) == 27) {
        break;
      }
    } else {
      this_thread::sleep_for(chrono::milliseconds(delay));
    }
  }
}
