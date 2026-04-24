#pragma once

#include <rapidjson/rapidjson.h>

#include <string>
#include <vector>

#include "utils/mapper.h"
#include "utils/switch.h"

using namespace std;

struct Config {
  struct Tracking {
    bool enabled = false;
    struct Preview {
      bool enabled = false;
      string window_name;
    } preview;
    int camera_id = 0;
    int retries = 0;
    int fps = 0;
    int width = 0;
    int height = 0;
    double brightness = 0.0;
    int radius = 0;
    int disabled_radius = 0;
    struct Center {
      int x = 0;
      int y = 0;
    } center;
    struct Formula {
      double a;
      double b;
      double c;
    } formula;
    struct Ball {
      bool setup;
      int min_area;
      vector<int> hsv_min;
      vector<int> hsv_max;
    } ball;

    struct Goal {
      struct Hsv {
        vector<int> hsv_min;
        vector<int> hsv_max;
      };

      string type;
      bool setup;
      int min_area;
      Hsv yellow;
      Hsv blue;
    } goal;
  } tracking;

  struct Serial {
    bool enabled = false;
    string device;
    int rate = 0;
    bool interference = false;
  } serial;

  struct Gpio {
    bool enabled = false;
    struct Buzzer {
      bool enabled = false;
      int pin = 0;
      int notes = 0;
    } buzzer;
    struct Buttons {
      bool enabled = false;
      vector<int> pins;
    } buttons;
    struct Display {
      bool enabled = false;
      string device;
      string address;
      string mode;
      int img = 0;
    } display;
  } gpio;

  struct Lidar {
    bool enabled = false;
    string path;
    struct Calibration {
      bool enabled;
      int delay;
      int threshold;
      double movement;
      double angle;
    } calibration;
  } lidar;

  struct Visualization {
    bool enabled = false;
    string window_name;
    int frames = 0;
    bool interactive = false;
  } visualization;

  struct Strategy {
    bool enabled;
    string role;
    double turn_precision;

    struct Predict {
      bool enabled;
      double alpha_xy;
    } predict;

    struct BallFilter {
      double alpha_xy;      // для позиции
      double beta_xy;       // для скорости
      double friction_tau;  // время остановки мяча от трения
      double latency_ms;
      double max_jump;  // максимальное перемещения мяча, при котором мы
                        // сбрасываем фильтр
      double lost_timeout_ms;
    } ball_filter;

    struct Motion {
      double max_linear_accel;
      double max_angular_accel;
      double decel_k;
      double wall_limit;
    } motion;

    struct Target {
      double x, y;
    };

    Mapper dribbling;
    double dribbling_slow;

    struct Attacker {
      double border;
      bool dubins_enabled;
    } attacker;

    struct Keeper {
      double global_border;
      double dubins_border;
      bool ram_enabled;
      struct Line {
        double padding;
        double y;
      } line;
    } keeper;

    struct Dubins {
      double bonus;
      double separate;
      double radius;
      double deep_inside;
      double camera_target_dist;
      Switch kick_precision;
      double aim_bonus;
      Mapper speed;
    } dubins;

    struct AttackerRicochet {
      bool enabled;
    } attacker_ricochet;

    Target target_left;
    Target target_right;
  } strategy;
};

extern Config config;

void load_config();
