#pragma once

class Button {
 private:
  const int pin = 8;

 public:
  bool was_pressed = false;
  void init();
  bool state();
};
