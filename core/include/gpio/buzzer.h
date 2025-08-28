#pragma once
#include "media/music.h"

class Buzzer {
   private:
    const int pin;
    void play_note(double, int, double);
    void rest(double);

   public:
    Buzzer(int);
    void play_song(const std::vector<Note> &);
    void beep();
    bool music_running = false;
    bool stop_music = false;
};
