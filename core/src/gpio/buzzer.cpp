#include "gpio/buzzer.h"

#include <softTone.h>
#include <wiringPi.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace std;

#define beat 500
#define cnat 16.35
#define csharp 17.32
#define dnat 18.35
#define eb 19.45
#define enat 20.60
#define fnat 21.83
#define fsharp 23.12
#define gnat 24.50
#define gsharp 25.96
#define anat 27.50
#define bb 29.14
#define bnat 30.87

struct Note {
    double base;
    int octave;
    double length;
};

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

Note notes[] = {
    {dnat, 4, 0.25},
    {dnat, 4, 0.25},
    {dnat, 5, 0.25},
    {0, 0, 0.25},
    {anat, 4, 0.25},
    {0, 0, 0.5},
    {gsharp, 4, 0.25},
    {0, 0, 0.25},
    {gnat, 4, 0.25},
    {0, 0, 0.25},
    {fnat, 4, 0.5},
    {dnat, 4, 0.25},
    {fnat, 4, 0.25},
    {gnat, 4, 0.25},

    {cnat, 4, 0.25},
    {cnat, 4, 0.25},
    {dnat, 5, 0.25},
    {0, 0, 0.25},
    {anat, 4, 0.25},
    {0, 0, 0.5},
    {gsharp, 4, 0.25},
    {0, 0, 0.25},
    {gnat, 4, 0.25},
    {0, 0, 0.25},
    {fnat, 4, 0.5},
    {dnat, 4, 0.25},
    {fnat, 4, 0.25},
    {gnat, 4, 0.25},

    {bnat, 3, 0.25},
    {bnat, 3, 0.25},
    {dnat, 5, 0.25},
    {0, 0, 0.25},
    {anat, 4, 0.25},
    {0, 0, 0.5},
    {gsharp, 4, 0.25},
    {0, 0, 0.25},
    {gnat, 4, 0.25},
    {0, 0, 0.25},
    {fnat, 4, 0.5},
    {dnat, 4, 0.25},
    {fnat, 4, 0.25},
    {gnat, 4, 0.25},

    {bb, 3, 0.25},
    {bb, 3, 0.25},
    {dnat, 5, 0.25},
    {0, 0, 0.25},
    {anat, 4, 0.25},
    {0, 0, 0.5},
    {gsharp, 4, 0.25},
    {0, 0, 0.25},
    {gnat, 4, 0.25},
    {0, 0, 0.25},
    {fnat, 4, 0.5},
    {dnat, 4, 0.25},
    {fnat, 4, 0.25},
    {gnat, 4, 0.25},

    // 5

    {dnat, 4, 0.25},
    {dnat, 4, 0.25},
    {dnat, 5, 0.25},
    {0, 0, 0.25},
    {anat, 4, 0.25},
    {0, 0, 0.5},
    {gsharp, 4, 0.25},
    {0, 0, 0.25},
    {gnat, 4, 0.25},
    {0, 0, 0.25},
    {fnat, 4, 0.5},
    {dnat, 4, 0.25},
    {fnat, 4, 0.25},
    {gnat, 4, 0.25},

    {cnat, 4, 0.25},
    {cnat, 4, 0.25},
    {dnat, 5, 0.25},
    {0, 0, 0.25},
    {anat, 4, 0.25},
    {0, 0, 0.5},
    {gsharp, 4, 0.25},
    {0, 0, 0.25},
    {gnat, 4, 0.25},
    {0, 0, 0.25},
    {fnat, 4, 0.5},
    {dnat, 4, 0.25},
    {fnat, 4, 0.25},
    {gnat, 4, 0.25},

    {bnat, 3, 0.25},
    {bnat, 3, 0.25},
    {dnat, 5, 0.25},
    {0, 0, 0.25},
    {anat, 4, 0.25},
    {0, 0, 0.5},
    {gsharp, 4, 0.25},
    {0, 0, 0.25},
    {gnat, 4, 0.25},
    {0, 0, 0.25},
    {fnat, 4, 0.5},
    {dnat, 4, 0.25},
    {fnat, 4, 0.25},
    {gnat, 4, 0.25},

    {bb, 3, 0.25},
    {bb, 3, 0.25},
    {dnat, 5, 0.25},
    {0, 0, 0.25},
    {anat, 4, 0.25},
    {0, 0, 0.5},
    {gsharp, 4, 0.25},
    {0, 0, 0.25},
    {gnat, 4, 0.25},
    {0, 0, 0.25},
    {fnat, 4, 0.5},
    {dnat, 4, 0.25},
    {fnat, 4, 0.25},
    {gnat, 4, 0.25},

    // 9

    {dnat, 5, 0.25},
    {dnat, 5, 0.25},
    {dnat, 6, 0.25},
    {0, 0, 0.25},
    {anat, 5, 0.25},
    {0, 0, 0.5},
    {gsharp, 5, 0.25},
    {0, 0, 0.25},
    {gnat, 5, 0.25},
    {0, 0, 0.25},
    {fnat, 5, 0.5},
    {dnat, 5, 0.25},
    {fnat, 5, 0.25},
    {gnat, 5, 0.25},

    {cnat, 5, 0.25},
    {cnat, 5, 0.25},
    {dnat, 6, 0.25},
    {0, 0, 0.25},
    {anat, 5, 0.25},
    {0, 0, 0.5},
    {gsharp, 5, 0.25},
    {0, 0, 0.25},
    {gnat, 5, 0.25},
    {0, 0, 0.25},
    {fnat, 5, 0.5},
    {dnat, 5, 0.25},
    {fnat, 5, 0.25},
    {gnat, 5, 0.25},

    {bnat, 4, 0.25},
    {bnat, 4, 0.25},
    {dnat, 6, 0.25},
    {0, 0, 0.25},
    {anat, 5, 0.25},
    {0, 0, 0.5},
    {gsharp, 5, 0.25},
    {0, 0, 0.25},
    {gnat, 5, 0.25},
    {0, 0, 0.25},
    {fnat, 5, 0.5},
    {dnat, 5, 0.25},
    {fnat, 5, 0.25},
    {gnat, 5, 0.25},

    {bb, 4, 0.25},
    {bb, 4, 0.25},
    {dnat, 6, 0.25},
    {0, 0, 0.25},
    {anat, 5, 0.25},
    {0, 0, 0.5},
    {gsharp, 5, 0.25},
    {0, 0, 0.25},
    {gnat, 5, 0.25},
    {0, 0, 0.25},
    {fnat, 5, 0.5},
    {dnat, 5, 0.25},
    {fnat, 5, 0.25},
    {gnat, 5, 0.25},

    {dnat, 5, 0.25},
    {dnat, 5, 0.25},
    {dnat, 6, 0.25},
    {0, 0, 0.25},
    {anat, 5, 0.25},
    {0, 0, 0.5},
    {gsharp, 5, 0.25},
    {0, 0, 0.25},
    {gnat, 5, 0.25},
    {0, 0, 0.25},
    {fnat, 5, 0.5},
    {dnat, 5, 0.25},
    {fnat, 5, 0.25},
    {gnat, 5, 0.25},

    {cnat, 5, 0.25},
    {cnat, 5, 0.25},
    {dnat, 6, 0.25},
    {0, 0, 0.25},
    {anat, 5, 0.25},
    {0, 0, 0.5},
    {gsharp, 5, 0.25},
    {0, 0, 0.25},
    {gnat, 5, 0.25},
    {0, 0, 0.25},
    {fnat, 5, 0.5},
    {dnat, 5, 0.25},
    {fnat, 5, 0.25},
    {gnat, 5, 0.25},

    {bnat, 4, 0.25},
    {bnat, 4, 0.25},
    {dnat, 6, 0.25},
    {0, 0, 0.25},
    {anat, 5, 0.25},
    {0, 0, 0.5},
    {gsharp, 5, 0.25},
    {0, 0, 0.25},
    {gnat, 5, 0.25},
    {0, 0, 0.25},
    {fnat, 5, 0.5},
    {dnat, 5, 0.25},
    {fnat, 5, 0.25},
    {gnat, 5, 0.25},

    {bb, 4, 0.25},
    {bb, 4, 0.25},
    {dnat, 6, 0.25},
    {0, 0, 0.25},
    {anat, 5, 0.25},
    {0, 0, 0.5},
    {gsharp, 5, 0.25},
    {0, 0, 0.25},
    {gnat, 5, 0.25},
    {0, 0, 0.25},
    {fnat, 5, 0.5},
    {dnat, 5, 0.25},
    {fnat, 5, 0.25},
    {gnat, 5, 0.25},

    // 13

    {fnat, 5, 0.5},
    {fnat, 5, 0.25},
    {fnat, 5, 0.25},
    {0, 0, 0.25},
    {fnat, 5, 0.25},
    {0, 0, 0.15},
    {enat, 5, 0.10},
    {fnat, 5, 0.5},
    {dnat, 5, 0.5},
    {dnat, 5, 1.25},

    {fnat, 5, 0.5},
    {fnat, 5, 0.25},
    {fnat, 5, 0.25},
    {0, 0, 0.25},
    {gnat, 5, 0.25},
    {0, 0, 0.25},
    {gsharp, 5, 0.5},
    {gnat, 5, 0.083},
    {gsharp, 5, 0.083},
    {gnat, 5, 0.084},
    {fnat, 5, 0.25},
    {dnat, 5, 0.25},
    {fnat, 5, 0.25},
    {gnat, 5, 0.25},
    {0, 0, 0.5},

    {fnat, 5, 0.5},
    {fnat, 5, 0.25},
    {fnat, 5, 0.25},
    {0, 0, 0.25},
    {gnat, 5, 0.25},
    {0, 0, 0.25},
    {gsharp, 5, 0.25},
    {0, 0, 0.25},
    {anat, 5, 0.25},
    {0, 0, 0.25},
    {cnat, 6, 0.25},
    {0, 0, 0.25},
    {anat, 5, 0.75},

    {dnat, 6, 0.25},
    {0, 0, 0.25},
    {dnat, 6, 0.25},
    {0, 0, 0.25},
    {dnat, 6, 0.25},
    {anat, 5, 0.25},
    {dnat, 6, 0.25},
    {cnat, 6, 1.0},

    // cycle
};

void Buzzer::megalovania() {
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
        cout << "Failed to setup pin" << endl;
        exit(-1);
    }
}

void Buzzer::beep() {
    softToneWrite(pin, 500);
    delay(200);
    softToneWrite(pin, 0);
}
