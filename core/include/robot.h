#pragma once

#include "uart.h"

using namespace LibSerial;

class Robot {
   private:
    UART uart;
    float angle = 0;
    bool emitter = false;
    bool kicker_charged = false;

    void read_from_arduino();
    void cycle();
    friend void start_cycle(Robot *);

   public:
    void start_arduino_reading();
};