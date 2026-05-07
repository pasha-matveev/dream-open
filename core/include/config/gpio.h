#pragma once

#include <rapidjson/fwd.h>

#include <memory>
#include <string>
#include <vector>

namespace cfg {

struct Gpio {
  Gpio(const rapidjson::Value&);
  ~Gpio();
  Gpio(const Gpio&) = delete;
  Gpio& operator=(const Gpio&) = delete;

  bool enabled;

  struct Buzzer {
    bool enabled;
    int pin;
    int notes;
    int duration;
  };
  std::unique_ptr<Buzzer> buzzer;

  struct Buttons {
    bool enabled;
    std::vector<int> pins;
  };
  std::unique_ptr<Buttons> buttons;

  struct Display {
    bool enabled;
    std::string device;
    std::string address;
    std::string mode;
    int img;
  };
  std::unique_ptr<Display> display;
};

}  // namespace cfg
