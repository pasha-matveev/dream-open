#include <spdlog/spdlog.h>

#include <chrono>
#include <thread>

#include "robot.h"
#include "strategy/field.h"
#include "strategy/strategy.h"
#include "strategy/visualization.h"
#include "tracking/object.h"
#include "utils/config.h"
#include "utils/millis.h"

using namespace std;

int main() {
  spdlog::info("Loading config...");
  load_config();
  spdlog::info("Config loaded");

  vector<Vec> field_points;

  // 11 7
  if (config.strategy.role == "keeper" || config.strategy.role == "challenge") {
    spdlog::info("Running as keeper");
    field_points = {{12, 12},   {12, 231},  {51, 231},  {51, 221},  {55, 213},
                    {66, 206},  {116, 206}, {127, 213}, {131, 221}, {131, 231},
                    {170, 231},

                    {170, 12},  {131, 12},  {131, 22},  {127, 30},  {116, 37},
                    {66, 37},   {55, 30},   {51, 22},   {51, 12}};
  } else {
    spdlog::info("Running as attacker");
    spdlog::info("Running as keeper");
    field_points = {{12, 80},   {12, 231},  {51, 231},  {51, 221},
                    {55, 213},  {66, 206},  {116, 206}, {127, 213},
                    {131, 221}, {131, 231}, {170, 231},

                    {170, 80}};
  }
  Field field(field_points);

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

  int delay = 1000 / config.strategy.fps;
  int read_time = 0;
  int strategy_time = 0;
  int write_time = 0;
  long long st = 0;

  auto compute_delay = [&](long long cycle_start) -> long long {
    long long elapsed = millis() - cycle_start;
    long long actual_delay = delay - elapsed;
    if (actual_delay <= 0) {
      spdlog::error(
          "Strategy FPS not achievable: cycle took {} out of {}. Read: {}; "
          "Strategy: {}; Write: {}",
          elapsed, delay, read_time, strategy_time, write_time);
      if (config.tracking.preview.enabled) {
        actual_delay = 1;
      } else {
        actual_delay = 0;
      }
    }
    return actual_delay;
  };

  while (true) {
    long long cycle_start = millis();
    st = millis();
    if (config.serial.enabled) {
      robot.read_from_arduino();
    }
    read_time = millis() - st;
    // ... strategy ...
    st = millis();
    strategy.run(robot, ball, goal, field);
    strategy_time = millis() - st;
    st = millis();
    if (config.serial.enabled) {
      robot.write_to_arduino();
    }
    write_time = millis() - st;
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
      if (cv::waitKey(compute_delay(cycle_start)) == 27) {
        break;
      }
    } else {
      this_thread::sleep_for(chrono::milliseconds(compute_delay(cycle_start)));
    }
  }
}
