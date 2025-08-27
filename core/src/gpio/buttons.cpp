#include "gpio/buttons.h"

#include <wiringPi.h>

#include <iostream>
#include <thread>

#include "gpio/buzzer.h"
#include "utils/config.h"

using namespace std;

void run_music(Buzzer *b) { b->megalovania(); }

void toggle_music(WPIWfiStatus status, void *p) {
    Buzzer *b = (Buzzer *)p;
    if (b->music_running) {
        b->stop_music = !b->stop_music;
    } else {
        thread th(run_music, b);
        th.detach();
    }
}

void setup_music_button(Buzzer *buzzer) {
    const int pin = config["gpio"]["buttons"][0].GetInt();
    pinMode(pin, INPUT);
    wiringPiISR2(pin, INT_EDGE_FALLING, toggle_music, 30, buzzer);
}

void setup_buttons(Buzzer *buzzer) { setup_music_button(buzzer); }