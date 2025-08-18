#include "Button.h"

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