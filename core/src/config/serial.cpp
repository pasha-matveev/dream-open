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
}