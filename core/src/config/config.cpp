#include "config/config.h"

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

#include <fstream>
#include <stdexcept>

#include "config/gpio.h"
#include "config/lidar.h"
#include "config/serial.h"
#include "config/strategy.h"
#include "config/tracking.h"
#include "config/visualization.h"

using namespace cfg;
using std::make_unique;

std::unique_ptr<Config> config;

Config::~Config() = default;

Config::Config(const rapidjson::Value& doc) {
  tracking = make_unique<Tracking>(doc["tracking"]);
  serial = make_unique<Serial>(doc["serial"]);
  gpio = make_unique<Gpio>(doc["gpio"]);
  lidar = make_unique<Lidar>(doc["lidar"]);
  visualization = make_unique<Visualization>(doc["visualization"]);
  strategy = make_unique<Strategy>(doc["strategy"]);
}

void load_config() {
  std::ifstream file("config.json");
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open config.json");
  }

  rapidjson::IStreamWrapper isw(file);
  rapidjson::Document doc;
  doc.ParseStream(isw);

  if (doc.HasParseError()) {
    throw std::runtime_error("Failed to parse config.json");
  }

  config = make_unique<Config>(doc);
}
