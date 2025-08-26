#pragma once

class Button {
   private:
    const int pin = 8;
    bool was_pressed = false;

   public:
    void init();
    bool state();
};
