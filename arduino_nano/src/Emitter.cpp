#include "Emitter.h"

#include "Robot.h"

void Emitter::init() { pinMode(pin, INPUT); }

void Emitter::read() { raw = analogRead(pin); }

void Emitter::reset() { raw = 0; }