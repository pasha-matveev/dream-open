#pragma once
#include "media/music.h"

class Buzzer {
   private:
    const int pin;
    long long beep_until = 0;
    bool beeping = false;
    void play_note(double, int, double);
    void rest(double);

   public:
    Buzzer(int);
    void play_song(const std::vector<Note> &);
    // Не вызывать одновременно с play_song(): оба пишут в один softTone-пин.
    void beep();
    void update();
    bool music_running = false;
    bool stop_music = false;
};
