#include "gpio/buttons.h"

#include <wiringPi.h>

#include <iostream>
#include <thread>

#include "gpio/buzzer.h"
#include "media/music.h"
#include "robot.h"
#include "utils/config.h"

using namespace std;

void run_music(Buzzer* b) { b->play_song(get_notes(1)); }

void toggle_music(WPIWfiStatus status, void* p) {
  Buzzer* b = (Buzzer*)p;
  if (b->music_running) {
    b->stop_music = !b->stop_music;
  } else {
    thread th(run_music, b);
    th.detach();
  }
}

void setup_music_button(Buzzer* buzzer) {
  if (buzzer == nullptr) return;
  const int pin = config.gpio.buttons.pins[0];
  pinMode(pin, INPUT);
  wiringPiISR2(pin, INT_EDGE_FALLING, toggle_music, 30, buzzer);
}

void handle_pause_button(WPIWfiStatus status, void* p) {
  Robot* robot = (Robot*)p;
  robot->pause = !robot->pause;
}

void setup_pause_button(Robot* robot) {
  assert(robot != nullptr);
  const int pin = config.gpio.buttons.pins[0];
  pinMode(pin, INPUT);
  wiringPiISR2(pin, INT_EDGE_FALLING, handle_pause_button, 250, robot);
}

void setup_buttons(Robot* robot) { setup_pause_button(robot); }
