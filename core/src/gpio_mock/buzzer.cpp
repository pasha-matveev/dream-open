#include "gpio/buzzer.h"

#include "media/music.h"

using namespace std;

void Buzzer::play_note(double note, int octave, double note_val) {}

void Buzzer::rest(double note_val) {}

void Buzzer::play_song(const vector<Note> &notes) {}

Buzzer::Buzzer(int pin) : pin(pin) {}

void Buzzer::beep() {}
