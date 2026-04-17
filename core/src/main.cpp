#include <spdlog/spdlog.h>

#include <chrono>
#include <csignal>
#include <thread>

#include "robot.h"
#include "strategy/field.h"
#include "strategy/strategy.h"
#include "strategy/visualization.h"
#include "tracking/object.h"
#include "utils/config.h"
#include "utils/geo/circle.h"
#include "utils/millis.h"

using namespace std;

volatile std::sig_atomic_t stop_requested = 0;

void handle_signal(int) { stop_requested = 1; }

constexpr int BDR = 20;

int main() {
  cout << "Program started" << endl;
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  spdlog::info("Loading config...");
  load_config();
  spdlog::info("Config loaded");

  vector<Vec> field_points;

  // 11 7
  if (config.strategy.role == "keeper" || config.strategy.role == "challenge") {
    spdlog::info("Running as keeper");
    const int AX = 38;
    const int BX = 45;
    const int CX = 53;
    const int CY = 36;
    const int MY = 70;
    field_points = {{AX, 20},       {AX, MY},       {182 - AX, MY},
                    {182 - AX, 20}, {182 - BX, 20}, {182 - CX, CY},
                    {CX, CY},       {BX, 20}};
  } else {
    spdlog::info("Running as attacker");
    field_points = {{BDR, BDR},
                    {BDR, 243 - BDR},
                    {51, 243 - BDR},
                    {51, 221},
                    {55, 213},
                    {66, 206},
                    {116, 206},
                    {127, 213},
                    {131, 221},
                    {131, 243 - BDR},
                    {182 - BDR, 243 - BDR},

                    {182 - BDR, BDR},
                    {182 - 12 - 35, BDR},
                    {182 - 12 - 35, 50},
                    {182 - 12 - 55, 60},
                    {12 + 55, 60},
                    {12 + 35, 50},
                    {12 + 35, BDR}};
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

  // Минимальная длительность цикла (мс) для интерактивной визуализации без
  // serial: ограничиваем частоту, чтобы не спинить CPU. С serial цикл всё
  // равно ограничен обменом с arduino.
  int min_period_ms = 1000 / config.visualization.frames;

  auto compute_delay = [&](long long cycle_start) -> long long {
    long long elapsed = millis() - cycle_start;
    long long actual_delay = min_period_ms - elapsed;
    if (actual_delay <= 0) {
      if (config.tracking.preview.enabled) {
        actual_delay = 1;
      } else {
        actual_delay = 0;
      }
    }
    return actual_delay;
  };

  while (!stop_requested) {
    if (config.visualization.enabled) {
      visualization->begin();
    }
    long long cycle_start = millis();
    if (config.serial.enabled) {
      robot.read_from_arduino();
    }
    strategy.run(robot, ball, goal, field);
    if (config.serial.enabled) {
      robot.write_to_arduino();
    }
    if (stop_requested) {
      break;
    }
    // visualisation
    if (config.visualization.enabled) {
      visualization->run(robot, ball, goal, field, strategy.get_last_dt());
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
    } else if (config.visualization.enabled) {
      auto sleep_ms = compute_delay(cycle_start);
      if (sleep_ms > 0) {
        this_thread::sleep_for(chrono::milliseconds(sleep_ms));
      }
    }
  }

  if (stop_requested) {
    spdlog::info("Termination signal received, shutting down...");
  }

  if (visualization != nullptr) {
    delete visualization;
  }
}
