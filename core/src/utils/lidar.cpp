#include "utils/lidar.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <csignal>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <sys/wait.h>
#include <unistd.h>

#include "robot.h"
#include "utils/config.h"
#include "utils/geo/vec.h"

double last_rotation = -4;

void LidarObject::update(double a, double d, double r, int w, int h) {
  angle = a;
  dist = d;
  rotation = r;
  width = w;
  height = h;
}

void LidarObject::rotate() {
  if (width < height) {
    swap(width, height);
    rotation -= M_PI / 2;
  }

  rotation = -rotation + M_PI / 2;

  rotation = normalize_angle2(rotation);
}

double LidarObject::get_radius() const {
  return sqrt(width * width + height * height) / 2.0;
}

void Lidar::start() {
  string cmd = config.lidar.path;

  if (running) {
    spdlog::warn("Lidar process already running");
    return;
  }

  int pipefd[2];
  if (::pipe(pipefd) == -1) {
    throw runtime_error("Failed to start lidar process");
  }

  pid_t pid = ::fork();
  if (pid == -1) {
    ::close(pipefd[0]);
    ::close(pipefd[1]);
    throw runtime_error("Failed to fork lidar process");
  }

  if (pid == 0) {
    // Child process: redirect stdout to pipe and exec command
    ::close(pipefd[0]);
    if (::dup2(pipefd[1], STDOUT_FILENO) == -1) {
      _exit(127);
    }
    ::close(pipefd[1]);
    execl("/bin/sh", "sh", "-c", cmd.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }

  ::close(pipefd[1]);
  FILE* stream = ::fdopen(pipefd[0], "r");
  if (!stream) {
    ::close(pipefd[0]);
    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);
    throw runtime_error("Failed to open lidar pipe stream");
  }

  pipe = stream;
  child_pid = pid;
  running = true;
  output_thread = thread(&Lidar::_output_loop, this);
}

void Lidar::stop() {
  running = false;

  if (child_pid > 0) {
    ::kill(child_pid, SIGTERM);
  }

  if (output_thread.joinable()) {
    output_thread.join();
  }

  if (pipe) {
    ::fclose(pipe);
    pipe = nullptr;
  }

  if (child_pid > 0) {
    ::waitpid(child_pid, nullptr, 0);
    child_pid = -1;
  }
}

Lidar::ComputeResult Lidar::compute(const Robot& robot) {
  vector<string> local_copy;

  {
    lock_guard<mutex> lock(data_mtx);
    if (latest_data.empty()) return {false, {0, 0}};
    local_copy = latest_data;
    latest_data.clear();
  }

  Vec v = {0, 0};
  bool computed = false;
  double result_rotation = 0;

  if (local_copy.size() >= 5) {
    field.update(stod(local_copy[0]), stod(local_copy[1]), stod(local_copy[2]),
                 stoi(local_copy[3]), stoi(local_copy[4]));
    field.rotate();

    double robot_angle = -field.rotation;
    double vector_angle = normalize_angle(robot_angle + field.angle - M_PI / 2);
    v = {(double)sin(vector_angle) * field.dist,
         (double)(-1.0 * cos(vector_angle) * field.dist)};
    result_rotation = -1 * field.rotation;

    computed = true;
  }

  obstacles_data.clear();
  for (size_t i = 5; i + 5 <= local_copy.size(); i += 5) {
    LidarObject obj;

    obj.update(stod(local_copy[i]), stod(local_copy[i + 1]),
               stod(local_copy[i + 2]), stoi(local_copy[i + 3]),
               stoi(local_copy[i + 4]));

    double result_angle =
        normalize_angle(robot.field_angle + obj.angle - (M_PI / 2));
    Vec vec{obj.dist * -1 * sin(result_angle), obj.dist * cos(result_angle)};
    Vec r = vec + robot.position;

    obstacles_data.push_back(r);
  }
  return {computed, v, result_rotation};
}

bool Lidar::new_data() {
  lock_guard<mutex> lock(data_mtx);
  return !latest_data.empty();
}

void Lidar::_output_loop() {
  char buffer[256];
  while (running && pipe && fgets(buffer, sizeof(buffer), pipe)) {
    string line(buffer);
    if (!line.empty() && line.back() == '\n') line.pop_back();

    istringstream iss(line);
    vector<string> tokens;
    string word;
    while (iss >> word) tokens.push_back(word);

    if (!tokens.empty() && tokens[0] == "Data") {
      received_data = true;
      tokens.erase(tokens.begin());
      {
        lock_guard<mutex> lock(data_mtx);
        latest_data = tokens;
      }
    } else {
      spdlog::warn("Failed to parse tokens");
      if (!tokens.empty()) {
        string message = "Tokens: ";
        for (size_t i = 0; i < tokens.size(); ++i) {
          message += tokens[i];
          if (i + 1 < tokens.size()) {
            message += ' ';
          }
        }
        spdlog::warn(message);
      }
    }
  }
}
