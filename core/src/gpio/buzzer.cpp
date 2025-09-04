#include "gpio/buzzer.h"

#include <softTone.h>
#include <wiringPi.h>

#include <chrono>
#include <iostream>
#include <thread>

#include "media/music.h"

using namespace std;

double note_to_octave(double note, int octave) { return note * (1 << octave); }

void Buzzer::play_note(double note, int octave, double note_val) {
    if (note > 0) {
        softToneWrite(pin, note_to_octave(note, octave));
        delay((beat * note_val) - 20);
        softToneWrite(pin, 0);
        delay(20);
    } else {
        delay(beat * note_val);
    }
}

void Buzzer::rest(double note_val) { delay(beat * note_val); }

void Buzzer::play_song(const vector<Note> &notes) {
    music_running = true;
    while (true) {
        for (const auto &n : notes) {
            while (stop_music) {
                std::this_thread::sleep_for(chrono::milliseconds(100));
            }
            play_note(n.base, n.octave, n.length);
        }
        std::this_thread::sleep_for(chrono::milliseconds(3000));
    }
}

Buzzer::Buzzer(int pin) : pin(pin) {
    if (softToneCreate(pin) == -1) {
        throw runtime_error("Failed to setup pin");
    }
}

void Buzzer::beep() {
    softToneWrite(pin, 500);
    delay(200);
    softToneWrite(pin, 0);
}
