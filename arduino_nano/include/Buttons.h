#include <Arduino.h>
#define ButtonPin1 4
#define ButtonPin2 8

class Buttons
{
  private:
    bool flag1 = 0;
    bool flag2 = 0;
    bool but1 = 0;
    bool but2 = 0;
  public:
    void init();
    bool Button1();
    bool Button2();
    bool state1();
    bool state2();
};

void Buttons::init() {
  pinMode(ButtonPin1, INPUT);
  pinMode(ButtonPin2, INPUT);
}

bool Buttons::Button1() {
  return digitalRead(ButtonPin1);
}

bool Buttons::Button2() {
  return digitalRead(ButtonPin2);
}

bool Buttons::state1() {
  if (Buttons::Button1()) {
    if (flag1) {
      flag1 = !flag1;
      but1 = !but1;
    }
  } else flag1 = 1;
  return but1;
}

bool Buttons::state2() {
  if (Buttons::Button2()) {
    if (flag2) {
      flag2 = 0;
      but2 = !but2;
    }
  } else flag2 = 1;
  return but2;
}
