#include <cmath>
#include <thread>

#include "config/config.h"
#include "config/strategy.h"
#include "config/strategy/meme.h"
#include "gpio/buzzer.h"
#include "media/music.h"
#include "robot.h"
#include "strategy/strategy.h"
#include "utils/geo/vec.h"
#include "utils/millis.h"

void Strategy::run_meme(Robot& robot) {
  auto& cfg = *config->strategy->meme;

  static bool music_started = false;
  if (!music_started && robot.buzzer != nullptr) {
    music_started = true;
    const auto& song = get_song_by_name(cfg.composition);
    Buzzer* b = robot.buzzer;
    std::thread([b, &song]() { b->play_song(song); }).detach();
  }

  static long long start_ms = -1;
  if (start_ms < 0) start_ms = millis();
  double t = (millis() - start_ms) * 1e-3;
  double omega = cfg.linear_speed / cfg.radius;
  double phase = omega * t;
  Vec dir{std::cos(phase), std::sin(phase)};
  robot.vel = dir * cfg.linear_speed;

  robot.rotation = M_PI;
  robot.rotation_limit = cfg.spin_speed;
}
