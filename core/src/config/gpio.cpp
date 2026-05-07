#include "config/gpio.h"

#include "config/to_vector.h"

using namespace cfg;
using std::make_unique;

Gpio::~Gpio() = default;
Gpio::Gpio(const rapidjson::Value& doc) {
  enabled = doc["enabled"].GetBool();

  buzzer = make_unique<Buzzer>();
  const rapidjson::Value& dbuzzer = doc["buzzer"];
  buzzer->enabled = dbuzzer["enabled"].GetBool();
  buzzer->pin = dbuzzer["pin"].GetInt();
  buzzer->notes = dbuzzer["notes"].GetInt();
  buzzer->duration = dbuzzer["duration"].GetInt();

  buttons = make_unique<Buttons>();
  const rapidjson::Value& dbuttons = doc["buttons"];
  buttons->enabled = dbuttons["enabled"].GetBool();
  buttons->pins = to_vector<int>(dbuttons["pins"]);

  display = make_unique<Display>();
  const rapidjson::Value& ddisplay = doc["display"];
  display->enabled = ddisplay["enabled"].GetBool();
  display->device = ddisplay["device"].GetString();
  display->address = ddisplay["address"].GetString();
  display->mode = ddisplay["mode"].GetString();
  display->img = ddisplay["img"].GetInt();
}
