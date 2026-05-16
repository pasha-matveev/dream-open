#include "config/config.h"

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <spdlog/spdlog.h>

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "config/gpio.h"
#include "config/lidar.h"
#include "config/serial.h"
#include "config/strategy.h"
#include "config/strategy/control.h"
#include "config/tracking.h"
#include "config/visualization.h"

using namespace cfg;
using std::make_unique;

std::unique_ptr<Config> config;

Config::~Config() = default;

namespace {

void apply_profile(Config& c, const std::string& name) {
  c.visualization->interactive = (name == "virtual");

  if (name == "manual") return;
  if (name == "play") {
    c.tracking->enabled = true;
    c.tracking->preview_enabled = false;
    c.serial->enabled = true;
    c.gpio->enabled = true;
    c.lidar->enabled = true;
    c.visualization->enabled = false;
    c.strategy->enabled = true;
  } else if (name == "visual") {
    c.tracking->enabled = true;
    c.tracking->preview_enabled = false;
    c.serial->enabled = true;
    c.gpio->enabled = true;
    c.lidar->enabled = true;
    c.visualization->enabled = true;
    c.strategy->enabled = true;
  } else if (name == "virtual") {
    c.tracking->enabled = false;
    c.tracking->preview_enabled = false;
    c.serial->enabled = false;
    c.gpio->enabled = false;
    c.lidar->enabled = false;
    c.visualization->enabled = true;
    c.strategy->enabled = true;
  } else if (name == "camera") {
    c.tracking->enabled = true;
    c.tracking->preview_enabled = true;
    c.serial->enabled = false;
    c.gpio->enabled = false;
    c.lidar->enabled = false;
    c.visualization->enabled = false;
    c.strategy->enabled = false;
  } else {
    throw std::runtime_error(
        "config.json: unknown profile '" + name +
        "' (expected one of: manual, play, visual, virtual, camera)");
  }
}

}  // namespace

Config::Config(const rapidjson::Value& doc) {
  std::string profile = doc["profile"].GetString();

  tracking = make_unique<Tracking>(doc["tracking"]);
  serial = make_unique<Serial>(doc["serial"]);
  gpio = make_unique<Gpio>(doc["gpio"]);
  lidar = make_unique<Lidar>(doc["lidar"]);
  visualization = make_unique<Visualization>(doc["visualization"]);
  strategy = make_unique<Strategy>(doc["strategy"]);

  apply_profile(*this, profile);
  spdlog::info("Profile: {}", profile);
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

namespace {

void set_hsv(rapidjson::Value& obj, const std::vector<int>& mn,
             const std::vector<int>& mx,
             rapidjson::Document::AllocatorType& a) {
  rapidjson::Value mn_arr(rapidjson::kArrayType);
  rapidjson::Value mx_arr(rapidjson::kArrayType);
  for (int v : mn) mn_arr.PushBack(v, a);
  for (int v : mx) mx_arr.PushBack(v, a);
  obj["hsv_min"] = mn_arr;
  obj["hsv_max"] = mx_arr;
}

}  // namespace

void save_config() {
  std::ifstream file("config.json");
  if (!file.is_open()) {
    spdlog::error("save_config: unable to open config.json for read");
    return;
  }

  rapidjson::IStreamWrapper isw(file);
  rapidjson::Document doc;
  doc.ParseStream(isw);
  file.close();

  if (doc.HasParseError()) {
    spdlog::error("save_config: failed to parse config.json");
    return;
  }

  auto& alloc = doc.GetAllocator();
  auto& tracking = doc["tracking"];

  set_hsv(tracking["ball"], config->tracking->ball->hsv_min,
          config->tracking->ball->hsv_max, alloc);
  set_hsv(tracking["goal"]["yellow"], config->tracking->goal->yellow->hsv_min,
          config->tracking->goal->yellow->hsv_max, alloc);
  set_hsv(tracking["goal"]["blue"], config->tracking->goal->blue->hsv_min,
          config->tracking->goal->blue->hsv_max, alloc);

  rapidjson::StringBuffer buf;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buf);
  writer.SetIndent(' ', 2);
  doc.Accept(writer);

  std::ofstream out("config.json.tmp");
  if (!out.is_open()) {
    spdlog::error("save_config: unable to open config.json.tmp for write");
    return;
  }
  out << buf.GetString();
  out.close();

  if (std::rename("config.json.tmp", "config.json") != 0) {
    spdlog::error("save_config: rename config.json.tmp -> config.json failed");
    return;
  }

  spdlog::info("Saved HSV calibration to config.json");
}
