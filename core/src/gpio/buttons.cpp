#include "gpio/buttons.h"

#include <spdlog/spdlog.h>
#include <wiringPi.h>

#include <iostream>
#include <thread>

#include "gpio/buzzer.h"
#include "media/music.h"
#include "robot.h"
#include "utils/config.h"

using namespace std;

static void run_music(Buzzer* b) { b->play_song(get_notes(1)); }

static void toggle_music(WPIWfiStatus status, void* p) {
  Buzzer* b = (Buzzer*)p;
  if (b->music_running) {
    b->stop_music = !b->stop_music;
  } else {
    thread th(run_music, b);
    th.detach();
  }
}

static void setup_music_button(Buzzer* buzzer) {
  if (buzzer == nullptr) return;
  const int pin = config.gpio.buttons.pins[0];
  pinMode(pin, INPUT);
  wiringPiISR2(pin, INT_EDGE_FALLING, toggle_music, 30, buzzer);
}

static void handle_pause_button(WPIWfiStatus status, void* p) {
  Robot* robot = (Robot*)p;
  if (robot->state == RobotState::PAUSE) {
    robot->state = RobotState::RUNNING;
    robot->look_forward();
  } else {
    robot->state = RobotState::PAUSE;
  }
}

static void setup_pause_button(Robot* robot) {
  assert(robot != nullptr);
  const int pin = config.gpio.buttons.pins[0];
  pinMode(pin, INPUT);
  wiringPiISR2(pin, INT_EDGE_FALLING, handle_pause_button, 250, robot);
}

static void handle_left_button(WPIWfiStatus status, void* p) {
  assert(p != nullptr);
  Robot* robot = (Robot*)p;
  robot->look_forward();
  if (robot->state != RobotState::PAUSE) {
    spdlog::error("Kickoff called from non-pause state {}", (int)robot->state);
    return;
  }
  robot->state = RobotState::KICKOFF_LEFT;
}

static void handle_right_button(WPIWfiStatus status, void* p) {
  assert(p != nullptr);
  Robot* robot = (Robot*)p;
  robot->look_forward();
  if (robot->state != RobotState::PAUSE) {
    spdlog::error("Kickoff called from non-pause state {}", (int)robot->state);
    return;
  }
  robot->state = RobotState::KICKOFF_RIGHT;
}

static void setup_left_button(Robot* robot) {
  assert(robot != nullptr);
  const int pin = config.gpio.buttons.pins[1];
  pinMode(pin, INPUT);
  wiringPiISR2(pin, INT_EDGE_FALLING, handle_left_button, 250, robot);
}

static void setup_right_button(Robot* robot) {
  assert(robot != nullptr);
  const int pin = config.gpio.buttons.pins[2];
  pinMode(pin, INPUT);
  wiringPiISR2(pin, INT_EDGE_FALLING, handle_right_button, 250, robot);
}

void setup_buttons(Robot* robot) {
  setup_pause_button(robot);
  setup_left_button(robot), setup_right_button(robot);
}
