#include "config/serial.h"

using namespace cfg;
using std::make_unique;

Serial::~Serial() = default;
Serial::Serial(const rapidjson::Value& doc) {
  enabled = doc["enabled"].GetBool();
  device = doc["device"].GetString();
  rate = doc["rate"].GetInt();
  interference = doc["interference"].GetBool();

  emitter = make_unique<Emitter>();
  const rapidjson::Value& demitter = doc["emitter"];
  emitter->threshold = demitter["threshold"].GetDouble();
  emitter->optimist = demitter["optimist"].GetInt();

  battery = make_unique<Battery>();
  const rapidjson::Value& dbattery = doc["battery"];
  battery->low_threshold = dbattery["low_threshold"].GetFloat();
  battery->warning_interval_ms = dbattery["warning_interval_ms"].GetInt();
  battery->pause_warning_interval_ms =
      dbattery["pause_warning_interval_ms"].GetInt();
}
