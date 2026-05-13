#pragma once

#include <rapidjson/fwd.h>

#include <memory>

namespace cfg {

struct Keeper {
  Keeper(const rapidjson::Value& doc);
  ~Keeper();
  Keeper(const Keeper&) = delete;
  Keeper& operator=(const Keeper&) = delete;

  // Y, выше которого вратарь стоит на проекции мяча на горизонтальную линию.
  // Ниже — на луче от центра ворот к мячу.
  double projection_border;

  struct Line {
    double padding;
    double y;
    // Минимальный Y, на который вратарь может встать при лучевой защите.
    // Когда точка пересечения луча с границей зоны имеет y < ray_min_y,
    // вратарь встаёт в крыло возле стойки на высоте ray_min_y.
    // Должен быть > 12 (выше нижнего отрезка зоны).
    double ray_min_y;
  };
  std::unique_ptr<Line> line;

  // Ram-режим
  struct Ram {
    bool enabled;
    // Минимальный зазор ball.y − robot.y
    double safety_y;
    // Триггер "далеко": дистанция до мяча ≥ этого значения, см.
    double far_min_dist;
    // Окно невидимости.
    long long blind_min_ms;
    // Пределы для drive_target в ram-режиме, см/с.
    double max_speed;
    double min_speed;
  };
  std::unique_ptr<Ram> ram;
};

}  // namespace cfg
