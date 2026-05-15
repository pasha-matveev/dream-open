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

  struct PushOut {
    double k;
    double v_min;
  };
  std::unique_ptr<PushOut> push_out;

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

  // Дополнительная зона ответственности вратаря: полоса вдоль задней линии
  // поля, в которую робот заехать не может (ворота / угловой карман у стенки),
  // но если мяч там — вратарь всё равно пытается доехать (field.apply прижмёт
  // к границе зоны).
  struct AdditionalResponsibility {
    // Верхняя граница полосы по Y (см). Учитывается, если ball.y < y_max.
    double y_max;
    // Отступ от боковых границ поля по X (см). Учитывается, если
    // x_padding ≤ ball.x ≤ FIELD_WIDTH − x_padding.
    double x_padding;
  };
  std::unique_ptr<AdditionalResponsibility> additional_responsibility;

  // Удар рикошетом, когда вратарь за проекцией своих ворот. Без него робот
  // при ударе вращался бы в сторону чужих ворот, держа мяч в лунке,
  // развёрнутой опасно — противник может перехватить.
  struct Ricochet {
    // Отступ от боковых стенок: x < x_padding → левый рикошет;
    // x > FIELD_WIDTH − x_padding → правый рикошет.
    double x_padding;
    // Гистерезис: выйти из ricochet-режима только когда x ушёл от порога
    // вглубь на `hysteresis` см.
    double hysteresis;
    // true — пересчитываем угол каждый тик от текущего ball_hole_position;
    // false — фиксируем угол при входе в режим (как kickoff).
    bool recompute_angle_each_tick;
    // Точки-таргеты для расчёта угла (зеркалятся через соответствующую
    // боковую стенку в compute_ricochet_field_angle).
    struct Target {
      double x;
      double y;
    };
    Target target_left;
    Target target_right;
  };
  std::unique_ptr<Ricochet> ricochet;
};

}  // namespace cfg
