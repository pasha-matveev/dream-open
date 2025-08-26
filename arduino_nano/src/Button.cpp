#include "Button.h"

#include <Arduino.h>

void Button::init() { pinMode(pin, INPUT); }

bool Button::state() {
    if (digitalRead(pin)) {
        was_pressed = true;
    }

    return was_pressed;
}