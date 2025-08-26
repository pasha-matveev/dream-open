#include "hardware/buzzer.h"

Buzzer::Buzzer(int pin) : pin(pin) { pinMode(pin, PWM_OUTPUT); }
void Buzzer::start_buzz() { digitalWrite(pin, HIGH); }
void Buzzer::stop_buzz() { digitalWrite(pin, LOW); }