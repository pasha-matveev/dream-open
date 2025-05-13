#include <Arduino.h>

class Kicker
{
private:
  unsigned long long charge_tm = 0;
  int min_delay = 500;
  int max_delay = 10000;
  int kick_pin = 7;
  int charge_pin = 4;
public:
  void init();
  void kick(int);
  void charge(bool);
  bool is_charged();
  void set_force(int);
  int force = 0;
};

void Kicker::init()
{
  pinMode(kick_pin, OUTPUT);
  pinMode(charge_pin, OUTPUT);
  digitalWrite(kick_pin, 0);
  digitalWrite(charge_pin, 0);
}

bool Kicker::is_charged()
{
  return charge_tm < millis();
}

void Kicker::set_force(int force)
{
  this->force = force;
}

void Kicker::kick(int power = -1)
{
  power = power == -1 ? force : power;
  if (power)
  {
    digitalWrite(kick_pin, 1);
    power = constrain(power, 1, 100);
    delayMicroseconds(map(power, 1, 100, min_delay, max_delay));
    digitalWrite(kick_pin, 0);
    charge_tm = millis() + (float)power * 31.38 * 3 + 1000;
    force = 0;
  }
}

void Kicker::charge(bool flag = true)
{
  if (is_charged() || !flag)
  {
    digitalWrite(charge_pin, 0);
  }
  else
  {
    digitalWrite(charge_pin, millis() % 3 == 0);
  }
}