#pragma once

#include <vector>

#include "strategy/field.h"
#include "utils/geo/polygon.h"
#include "utils/geo/vec.h"

// Высота физической зоны вратаря (верхний край допустимой области, см).
constexpr double KEEPER_ZONE_TOTAL_HEIGHT = 63.0;
// Расстояние от боковой границы поля до боковой границы keeper-зоны (см).
constexpr double KEEPER_ZONE_SIDE_DIST = 36.0;

// Вырез под ворота (cutout): робот туда заехать не может (там физически
// ворота), но мяч там может оказаться. Если мяч в этом прямоугольнике —
// пытаемся его всё равно забрать (заезжаем настолько глубоко, насколько
// разрешит field.apply()).
constexpr double KEEPER_GOAL_CUTOUT_X_MIN = 51.0;
constexpr double KEEPER_GOAL_CUTOUT_X_MAX = FIELD_WIDTH - 51.0;  // 131
constexpr double KEEPER_GOAL_CUTOUT_Y_MAX = 37.0;

// Точки физической зоны вратаря: прямоугольник 36..146 x 12..63 с нотчем
// возле ворот (cutout 80x25 с R15 угловыми кривыми). Включает обе нижние
// особые точки {52,57} и {130,57}.
std::vector<Vec> keeper_zone_points();

// Точки физической зоны нападающего: верхняя часть поля (выше keeper-зоны).
// Нижняя грань поднята до KEEPER_ZONE_TOTAL_HEIGHT + safety, чтобы нападающий
// не мог заехать в зону вратаря (Variant A анти-коллизии).
std::vector<Vec> attacker_zone_points();

// Удобные обёртки, возвращающие готовые Polygon (нужны там, где требуется
// hyst_inside / inside / find_intersection).
Polygon make_keeper_zone();
Polygon make_attacker_zone();
