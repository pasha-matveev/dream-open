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
};

}  // namespace cfg
