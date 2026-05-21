#include "gpio/buzzer.h"

#include <softTone.h>
#include <wiringPi.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

#include "config/config.h"
#include "config/gpio.h"
#include "media/music.h"

using namespace std;

// wiringPi.h declares its own `millis()` returning `unsigned int`, so we can't
// use the project-wide `long long millis()` here. Local helper avoids the
// clash.
static long long now_ms() {
  return chrono::duration_cast<chrono::milliseconds>(
             chrono::steady_clock::now().time_since_epoch())
      .count();
}

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

void Buzzer::play_song(const vector<Note>& notes) {
  music_running = true;
  while (true) {
    for (const auto& n : notes) {
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
  if (!beeping) {
    beep_until = now_ms() + config->gpio->buzzer->duration;
    softToneWrite(pin, 500);
    beeping = true;
  }
  beep_until = max(beep_until, now_ms() + 10);
}

void Buzzer::update() {
  if (beeping && now_ms() >= beep_until) {
    softToneWrite(pin, 0);
    beeping = false;
  }
}

void Buzzer::play_chirps(const std::vector<Chirp>& chirps) {
  const int my_gen = ++chirp_gen_;
  constexpr int step_ms = 5;
  for (const auto& c : chirps) {
    if (chirp_gen_.load() != my_gen) return;
    if (c.f_start == 0) {
      softToneWrite(pin, 0);
      delay(c.ms);
      continue;
    }
    const int n = std::max(1, c.ms / step_ms);
    const int denom = std::max(1, n - 1);
    for (int i = 0; i < n; ++i) {
      if (chirp_gen_.load() != my_gen) return;
      const int freq = c.f_start + (c.f_end - c.f_start) * i / denom;
      softToneWrite(pin, freq);
      delay(step_ms);
    }
  }
  if (chirp_gen_.load() == my_gen) softToneWrite(pin, 0);
}

void Buzzer::play_chirps_async(const std::vector<Chirp>& chirps) {
  std::thread([this, chirps] { play_chirps(chirps); }).detach();
}
