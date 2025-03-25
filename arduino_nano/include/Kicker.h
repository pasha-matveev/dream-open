#include <Arduino.h>

class Kicker
{
private:
  unsigned long long kick_tm = 0;
  int min_delay = 500;
  int max_delay = 10000;
  int pin = 9;
public:
  void init();
  void kick(int);
  bool is_ready();
  int force = 0;
};

void Kicker::init()
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, 0);
}

bool Kicker::is_ready()
{
  return kick_tm < millis();
}

void Kicker::kick(int power)
{
  if (is_ready() && power)
  {
    digitalWrite(pin, 1);
    power = constrain(power, 1, 100);
    delayMicroseconds(map(power, 1, 100, min_delay, max_delay));
    digitalWrite(pin, 0);
    kick_tm = millis() + (float)power * 31.38 + 3500;
  }
}