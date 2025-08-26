#pragma once

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
