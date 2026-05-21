#pragma once
#include <atomic>
#include <vector>

#include "media/music.h"

struct Chirp {
    int f_start;  // Hz, 0 = silence (rest)
    int f_end;    // Hz, == f_start = постоянный тон
    int ms;
};

class Buzzer {
   private:
    const int pin;
    long long beep_until = 0;
    bool beeping = false;
    std::atomic<int> chirp_gen_{0};
    void play_note(double, int, double);
    void rest(double);

   public:
    Buzzer(int);
    void play_song(const std::vector<Note> &);
    // Проигрывает последовательность свипов один раз. _async кидает работу в
    // отдельный тред и сразу возвращается. Если вызвать ещё раз, пока играет
    // предыдущий — старый обрывается, начинает играть новый.
    void play_chirps(const std::vector<Chirp> &chirps);
    void play_chirps_async(const std::vector<Chirp> &chirps);
    // Не вызывать одновременно с play_song(): оба пишут в один softTone-пин.
    void beep();
    void update();
    bool music_running = false;
    bool stop_music = false;
};
