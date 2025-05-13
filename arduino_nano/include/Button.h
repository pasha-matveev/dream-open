#include <Arduino.h>

class Button
{
private:
  int pin = 8;
  bool flag = 0;
  bool but = 0;

public:
  void init();
  bool state();
};

void Button::init()
{
  pinMode(pin, INPUT);
}

bool Button::state()
{
  if (digitalRead(pin))
  {
    if (flag)
    {
      flag = !flag;
      but = !but;
    }
  }
  else
    flag = 1;
  return but;
}
