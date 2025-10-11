#include <atomic>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "config.h"
#include "utils/vec.h"

using namespace std;

class LidarObject {
 public:
  double angle = 0;
  double dist = 0;
  double rotation = 0;
  int width = 0;
  int height = 0;

  void update(double a, double d, double r, int w, int h);

  void rotate();

  double get_radius() const;
};

class Lidar {
 public:
  struct ComputeResult {
    bool computed;
    Vec v;
    double rotation;
  };

  void start();

  void stop();

  ComputeResult compute();

  bool new_data();

  ~Lidar() { stop(); }

  LidarObject field;
  vector<LidarObject> obstacles_data;
  bool received_data = false;

 private:
  FILE* pipe = nullptr;
  thread output_thread;
  atomic<bool> running{false};

  mutex data_mtx;
  vector<string> latest_data;

  void _output_loop();
};
