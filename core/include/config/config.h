#pragma once

#include <rapidjson/fwd.h>

#include <memory>

namespace cfg {

struct Tracking;
struct Serial;
struct Gpio;
struct Lidar;
struct Visualization;
struct Strategy;

struct Config {
  Config() = delete;
  Config(const rapidjson::Value&);
  ~Config();
  Config(const Config&) = delete;
  Config& operator=(const Config&) = delete;

  std::unique_ptr<cfg::Tracking> tracking;
  std::unique_ptr<cfg::Serial> serial;
  std::unique_ptr<cfg::Gpio> gpio;
  std::unique_ptr<cfg::Lidar> lidar;
  std::unique_ptr<cfg::Visualization> visualization;
  std::unique_ptr<cfg::Strategy> strategy;
};

}  // namespace cfg

extern std::unique_ptr<cfg::Config> config;

void load_config();
