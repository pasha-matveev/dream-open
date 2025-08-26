#pragma once

#include <wiringPi.h>

using namespace std;

class Buzzer {
   private:
    const int pin;
    void play_note(double, int, double);
    void rest(double);

   public:
    Buzzer(int);
    void megalovania();
    void beep();
    bool music_running = false;
    bool stop_music = false;
};

void toggle_music(WPIWfiStatus status, void *p);