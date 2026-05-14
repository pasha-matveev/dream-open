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
#include "strategy/zones.h"
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

  const bool is_keeper = config->strategy->role == "keeper";
  spdlog::info("Running as {}", is_keeper ? "keeper" : "attacker");
  Field field = is_keeper
                    ? Field(keeper_zone_points(), keeper_zone_brake_flags())
                    : Field(attacker_zone_points());

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
  // Свои ворота — противоположный цвет, нужны для калибровки гироскопа в PAUSE.
  Object own_goal(config->tracking->goal->type == "yellow"
                      ? config->tracking->goal->blue->hsv_min
                      : config->tracking->goal->yellow->hsv_min,
                  config->tracking->goal->type == "yellow"
                      ? config->tracking->goal->blue->hsv_max
                      : config->tracking->goal->yellow->hsv_max,
                  /*setup=*/false, config->tracking->goal->min_area);
  Robot robot;
  spdlog::info("Initializing hardware...");
  robot.init_hardware(ball, goal, own_goal);
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
    strategy.run(robot, ball, goal, own_goal, field);
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
