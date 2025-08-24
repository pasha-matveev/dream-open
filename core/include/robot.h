#pragma once

#include "uart.h"

using namespace LibSerial;

class Robot {
   private:
    UART uart;

    float angle = 0;
    bool emitter = false;
    bool kicker_charged = false;

    float direction = 0;
    float speed = 0;
    float rotation = 0;
    float rotation_limit = 0;
    int dribling = 0;
    int kicked_force;

    void read_from_arduino();
    void cycle();
    friend void start_cycle(Robot *);

   public:
    void start_arduino_reading();
    void write_to_arduino();
};