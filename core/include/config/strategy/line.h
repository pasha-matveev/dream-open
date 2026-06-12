#pragma once

#include <rapidjson/fwd.h>

namespace cfg {

// Параметры режима езды по линии (прикол на чужой трассе line-follower).
// Все поля обязательны. Детектор сэмплит кольцо радиуса ring_radius_px на
// hsv-кадре; «чёрный» сэмпл — средний V по окрестности <= v_threshold.
struct Line {
  Line(const rapidjson::Value& doc);
  ~Line();
  Line(const Line&) = delete;
  Line& operator=(const Line&) = delete;

  double speed;          // линейная скорость, см/с
  double rotation_gain;  // демпфирование доворота: rotation = gain * θ, (0..1]
  double rotation_limit; // ограничение скорости доворота корпуса, рад/с
  int ring_radius_px;    // радиус кольца сэмплирования в пикселях
  int v_threshold;       // порог яркости V (0..255): <= → чёрное
  int samples;           // число угловых сэмплов по кольцу
  double min_arc_deg;    // мин. угловая ширина дуги (отсечь шум)
  double max_arc_deg;    // макс. угловая ширина дуги (отсечь тело/тень)
  double max_turn_deg;   // конус вокруг «вперёд» (θ=0) для выбора кандидата
  int lost_timeout_ms;   // сколько коастить прямо после потери линии до стопа
};

}  // namespace cfg
