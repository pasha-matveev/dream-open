#include <spdlog/spdlog.h>

#include <chrono>
#include <csignal>
#include <thread>

#include "config/config.h"
#include "config/serial.h"
#include "config/strategy.h"
#include "config/tracking.h"
#include "config/visualization.h"
#include "robot.h"
#include "strategy/field.h"
#include "strategy/strategy.h"
#include "strategy/visualization.h"
#include "tracking/object.h"
#include "utils/geo/circle.h"
#include "utils/millis.h"

using namespace std;

volatile std::sig_atomic_t stop_requested = 0;

void handle_signal(int) { stop_requested = 1; }

constexpr int BDR = 12;

int main() {
  cout << "Program started" << endl;
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  spdlog::info("Loading config...");
  load_config();
  spdlog::info("Config loaded");

  if (config->tracking->ball->setup && config->tracking->goal->setup) {
    spdlog::error(
        "Both ball.setup and goal.setup are true; trackbars share window "
        "'Camera' and would conflict. Calibrate one at a time.");
    return 1;
  }

  vector<Vec> field_points;

  // 11 7
  if (config->strategy->role == "keeper" ||
      config->strategy->role == "challenge") {
    spdlog::info("Running as keeper");
    const int AX = 38;
    const int BX = 45;
    const int CX = 53;
    const int CY = 36;
    const int MY = 70;
    field_points = {{AX, 20},       {AX, MY},       {182 - AX, MY},
                    {182 - AX, 20}, {182 - BX, 20}, {182 - CX, CY},
                    {CX, CY},       {BX, 20}};

    // field_points = {{BDR, BDR},
    //                 {BDR, 243 - BDR},
    //                 {51, 243 - BDR},
    //                 {51, 221},
    //                 {55, 213},
    //                 {66, 206},
    //                 {116, 206},
    //                 {127, 213},
    //                 {131, 221},
    //                 {131, 243 - BDR},
    //                 {182 - BDR, 243 - BDR},

    //                 {182 - BDR, BDR},
    //                 {182 - 12 - 35, BDR},
    //                 {182 - 12 - 35, 30},
    //                 {182 - 12 - 55, 40},
    //                 {12 + 55, 40},
    //                 {12 + 35, 30},
    //                 {12 + 35, BDR}};
  } else {
    spdlog::info("Running as attacker");
    field_points = {{12.0, 12.0},
                    {12.0, 231.0},
                    // top cutout, 80x25 with R15 inner corners
                    {51.0, 231.0},
                    {51.0, 221.0},
                    {52.1, 215.3},
                    {55.4, 210.4},
                    {60.3, 207.1},
                    {66.0, 206.0},
                    {116.0, 206.0},
                    {121.7, 207.1},
                    {126.6, 210.4},
                    {129.9, 215.3},
                    {131.0, 221.0},
                    {131.0, 231.0},
                    {170.0, 231.0},
                    {170.0, 12.0},
                    // bottom cutout, 80x25 with R15 inner corners
                    {131.0, 12.0},
                    {131.0, 22.0},
                    {129.9, 27.7},
                    {126.6, 32.6},
                    {121.7, 35.9},
                    {116.0, 37.0},
                    {66.0, 37.0},
                    {60.3, 35.9},
                    {55.4, 32.6},
                    {52.1, 27.7},
                    {51.0, 22.0},
                    {51.0, 12.0}};
  }
  Field field(field_points);

  if (config->tracking->preview_enabled) {
    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
  }

  Object ball(config->tracking->ball->hsv_min, config->tracking->ball->hsv_max,
              config->tracking->ball->setup, config->tracking->ball->min_area);
  Object goal(config->tracking->goal->type == "yellow"
                  ? config->tracking->goal->yellow->hsv_min
                  : config->tracking->goal->blue->hsv_min,
              config->tracking->goal->type == "yellow"
                  ? config->tracking->goal->yellow->hsv_max
                  : config->tracking->goal->blue->hsv_max,
              config->tracking->goal->setup, config->tracking->goal->min_area);
  Robot robot;
  spdlog::info("Initializing hardware...");
  robot.init_hardware(ball, goal);
  spdlog::info("Hardware ready");

  Strategy strategy;

  Visualization* visualization = nullptr;

  if (config->visualization->enabled) {
    visualization = new Visualization();
  }

  // Минимальная длительность цикла (мс) для интерактивной визуализации без
  // serial: ограничиваем частоту, чтобы не спинить CPU. С serial цикл всё
  // равно ограничен обменом с arduino.
  int min_period_ms = 1000 / config->visualization->frames;

  auto compute_delay = [&](long long cycle_start) -> long long {
    long long elapsed = millis() - cycle_start;
    long long actual_delay = min_period_ms - elapsed;
    if (actual_delay <= 0) {
      if (config->tracking->preview_enabled) {
        actual_delay = 1;
      } else {
        actual_delay = 0;
      }
    }
    return actual_delay;
  };

  while (!stop_requested) {
    if (config->visualization->enabled) {
      visualization->begin();
    }
    long long cycle_start = millis();
    if (config->serial->enabled) {
      robot.read_from_arduino();
    }
    strategy.run(robot, ball, goal, field);
    robot.update_buzzer();
    if (config->serial->enabled) {
      robot.write_to_arduino();
    }
    if (stop_requested) {
      break;
    }
    // visualisation
    if (config->visualization->enabled) {
      visualization->run(robot, ball, goal, field, strategy.get_last_dt());
      if (visualization->closed) {
        break;
      }
    }
    // delay
    if (config->tracking->preview_enabled) {
      try {
        robot.camera->show_preview();
      } catch (...) {
      }
      int key = cv::waitKey(compute_delay(cycle_start));
      if (key == 27) {
        break;
      }
      if (key == 's' &&
          (config->tracking->ball->setup || config->tracking->goal->setup)) {
        if (config->tracking->ball->setup) {
          ball.sync_to(config->tracking->ball->hsv_min,
                       config->tracking->ball->hsv_max);
        }
        if (config->tracking->goal->setup) {
          auto& g = config->tracking->goal->type == "yellow"
                        ? *config->tracking->goal->yellow
                        : *config->tracking->goal->blue;
          goal.sync_to(g.hsv_min, g.hsv_max);
        }
        save_config();
      }
    } else if (config->visualization->enabled) {
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
