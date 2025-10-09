#include <spdlog/spdlog.h>

#include <chrono>
#include <thread>

#include "robot.h"
#include "strategy/strategy.h"
#include "strategy/visualization.h"
#include "tracking/ball.h"
#include "utils/config.h"

using namespace std;

int main() {
  spdlog::info("Loading config...");
  load_config();
  spdlog::info("Config loaded");

  if (config["tracking"]["preview"]["enabled"].GetBool()) {
    cv::namedWindow(config["tracking"]["preview"]["window_name"].GetString(),
                    cv::WINDOW_AUTOSIZE);
  }

  Ball ball(make_int_vector(config["tracking"]["ball"]["hsv_min"].GetArray()),
            make_int_vector(config["tracking"]["ball"]["hsv_max"].GetArray()),
            config["tracking"]["preview"]["setup"].GetBool());

  Robot robot;
  spdlog::info("Initializing hardware...");
  robot.init_hardware(ball);
  spdlog::info("Hardware ready");

  Strategy strategy;

  Visualization* visualization = nullptr;

  if (config["visualization"]["enabled"].GetBool()) {
    visualization = new Visualization();
  }

  int delay = 1000 / config["tracking"]["fps"].GetInt() / 2;
  while (true) {
    if (config["serial"]["enabled"].GetBool()) {
      robot.read_from_arduino();
    }
    // ... strategy ...
    strategy.run(robot, ball);
    if (config["serial"]["enabled"].GetBool()) {
      robot.write_to_arduino();
    }
    // visualisation
    if (config["visualization"]["enabled"].GetBool()) {
      visualization->run(robot, ball);
      if (visualization->closed) {
        break;
      }
    }
    // delay
    if (config["tracking"]["preview"]["enabled"].GetBool()) {
      try {
        robot.camera->show_preview();
      } catch (...) {
      }
      if (cv::waitKey(delay) == 27) {
        break;
      }
    } else {
      this_thread::sleep_for(chrono::milliseconds(delay));
    }
  }
}
