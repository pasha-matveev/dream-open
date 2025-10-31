#pragma once

#include <string>
#include <vector>

using namespace std;

struct Config {
  struct Tracking {
    bool enabled = false;
    struct Preview {
      bool enabled = false;
      string window_name;
      bool setup = false;
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
    struct Ball {
      vector<int> hsv_min;
      vector<int> hsv_max;
    } ball;
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
  } lidar;

  struct Visualization {
    bool enabled = false;
    string window_name;
    int frames = 0;
    bool interactive = false;
  } visualization;

  struct Strategy {
    string role;
    int fps;
  } strategy;
};

extern Config config;

void load_config();
