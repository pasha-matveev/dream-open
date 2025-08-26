#pragma once

#include <wiringPi.h>

class Buzzer {
   private:
    const int pin;

   public:
   Buzzer(int);
   void start_buzz();
   void stop_buzz();
};