#include "Button.h"

#include <Arduino.h>

void Button::init() { pinMode(pin, INPUT); }

bool Button::state() {
    if (digitalRead(pin)) {
        if (flag) {
            flag = false;
            but = true;
        }
    } else {
        flag = false;
    }

    return but;
}