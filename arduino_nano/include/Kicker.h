#include <Arduino.h>

class Kicker
{
private:
  unsigned long long kick_tm = 0;
  int min_delay = 500;
  int max_delay = 10000;
  int kick_pin = 7;
  int charge_pin = 4;
public:
  void init();
  void kick(int);
  bool is_ready();
  int force = 0;
};

void Kicker::init()
{
  pinMode(kick_pin, OUTPUT);
  pinMode(charge_pin, OUTPUT);
  digitalWrite(kick_pin, 0);
  digitalWrite(charge_pin, 0);
  kick_tm = millis() + 5000;
}

bool Kicker::is_ready()
{
  return kick_tm < millis();
}

void Kicker::kick(int power)
{
  if (is_ready() && power)
  {
    digitalWrite(kick_pin, 1);
    power = constrain(power, 1, 100);
    delayMicroseconds(map(power, 1, 100, min_delay, max_delay));
    digitalWrite(kick_pin, 0);
    kick_tm = millis() + (float)power * 31.38 + 3500;
  }

  if (is_ready())
  {
    digitalWrite(charge_pin, 0);
  }
  else
  {
    digitalWrite(charge_pin, 1);
  }
}