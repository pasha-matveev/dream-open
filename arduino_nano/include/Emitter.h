#include <Arduino.h>

class Emitter
{
private:
  int pin = A0;
public:
  void read();
  void init();
  void reset();
  int raw = 0;
  bool val = 0;
  bool first = 1;
  unsigned long long first_tm = 0;
  unsigned long long last_tm = 0;
};

void Emitter::init()
{
  pinMode(pin, INPUT);
}

void Emitter::read()
{
  raw = analogRead(pin);
  val = ROBOT ? (raw < 180) : (raw < 450);
  if (val)
    last_tm = millis();
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

void Emitter::reset()
{
  first = 1;
  first_tm = millis();
  val = 0;
  raw = 0;
}