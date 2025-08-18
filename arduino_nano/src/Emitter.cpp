#include "Emitter.h"
#include "Robot.h"

void Emitter::init()
{
  pinMode(pin, INPUT);
}

void Emitter::read()
{
  raw = analogRead(pin);
  val = ROBOT ? (raw < 400) : (raw < 25);
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