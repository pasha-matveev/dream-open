#include "Kicker.h"

void Kicker::init() {
  pinMode(kick_pin, OUTPUT);
  pinMode(charge_pin, OUTPUT);
  digitalWrite(kick_pin, 0);
  digitalWrite(charge_pin, 0);
}

bool Kicker::is_charged() { return charge_tm < millis(); }

void Kicker::kick(int power) {
  if (power && kick_tm < millis()) {
    digitalWrite(charge_pin, 0);
    delay(1);
    digitalWrite(kick_pin, 1);
    power = constrain(power, 1, 100);
    delayMicroseconds(map(power, 1, 100, min_delay, max_delay));
    digitalWrite(kick_pin, 0);
    delay(1);
    charge_tm = millis() + 3000;
    kick_tm = millis() + min_kick_delay;
  }
}

void Kicker::charge(bool flag) {
  if (is_charged() || !flag) {
    digitalWrite(charge_pin, 0);
  } else {
    digitalWrite(charge_pin, millis() % 2 == 0);
  }
}

void Kicker::start() {
  if (started()) return;
  charge_tm = millis() + 3000;
}

bool Kicker::started() { return charge_tm != 0; }