#pragma once

#include "utils/uart.h"

using namespace LibSerial;

class Robot {
   private:
    UART uart;

   public:
    float angle = 0;
    bool emitter = false;
    bool kicker_charged = false;

    float direction = 0;
    float speed = 0;
    float rotation = 0;
    float rotation_limit = 30;
    int dribling = 0;
    int kicker_force = 0;

    void init_uart();
    void read_from_arduino();
    void write_to_arduino();
};