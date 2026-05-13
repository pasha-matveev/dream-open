#pragma once

#include <vector>

#include "utils/geo/polygon.h"
#include "utils/geo/vec.h"

// Высота физической зоны вратаря (верхний край допустимой области, см).
constexpr double KEEPER_ZONE_TOTAL_HEIGHT = 63.0;
// Расстояние от боковой границы поля до боковой границы keeper-зоны (см).
constexpr double KEEPER_ZONE_SIDE_DIST = 36.0;

// Дополнительная зона ответственности вратаря: полоса вдоль задней линии
// (y < предела), куда робот заехать не может (там физически ворота либо
// угловой карман у стенки), но мяч там может оказаться. Если мяч в этой
// полосе — пытаемся его всё равно забрать (заезжаем настолько глубоко,
// насколько разрешит field.apply()).
constexpr double KEEPER_ADDITIONAL_RESPONSIBILITY_Y_MAX = 37.0;

// Точки физической зоны вратаря: прямоугольник 36..146 x 12..63 с нотчем
// возле ворот (cutout 80x25 с R15 угловыми кривыми). Включает обе нижние
// особые точки {52,57} и {130,57}.
std::vector<Vec> keeper_zone_points();

// Точки физической зоны нападающего: верхняя часть поля (выше keeper-зоны).
// Нижняя грань поднята до KEEPER_ZONE_TOTAL_HEIGHT + safety, чтобы нападающий
// не мог заехать в зону вратаря (Variant A анти-коллизии).
std::vector<Vec> attacker_zone_points();

// Точки «зоны ответственности вратаря» — простой прямоугольник, охватывающий
// keeper-зону + ворота с запасом. Используется нападающим, чтобы решить
// «мяч у вратаря — отъезжаю». Все точки keeper_zone лежат строго внутри
// (с запасом ATTACKER_BOTTOM_SAFETY=2см), так что гистерезис hyst_inside
// не флипает на крайних точках.
std::vector<Vec> keeper_responsibility_points();

// Удобные обёртки, возвращающие готовые Polygon (нужны там, где требуется
// hyst_inside / inside / find_intersection).
Polygon make_keeper_zone();
Polygon make_attacker_zone();
Polygon make_keeper_responsibility();
