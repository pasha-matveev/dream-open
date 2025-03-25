#include <Arduino.h>

class Emitter
{
private:
  int pin = A0;
public:
  void read();
  void init();
  int raw = 0;
  bool val = 0;
  bool first = 1;
  unsigned long long first_tm = 0;
};

void Emitter::init()
{
  pinMode(pin, INPUT);
}

void Emitter::read()
{
  raw = analogRead(pin);
  val = raw < 880;
  if (val)
  {
    if (first)
      first_tm = millis();
    first = 0;
  }
  else
  {
    first_tm = millis();
    first = 1;
  }
}