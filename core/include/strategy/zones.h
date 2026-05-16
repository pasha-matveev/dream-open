#pragma once

#include <vector>

#include "strategy/brake_mode.h"
#include "utils/geo/polygon.h"
#include "utils/geo/vec.h"

// Высота физической зоны вратаря (верхний край допустимой области, см).
constexpr double KEEPER_ZONE_TOTAL_HEIGHT = 63.0;
// Расстояние от боковой границы поля до боковой границы keeper-зоны (см).
constexpr double KEEPER_ZONE_SIDE_DIST = 36.0;
constexpr double KEEPER_ADDITIONAL_DIST = 30;

// Точки физической зоны вратаря: прямоугольник 36..146 x 12..63 с нотчем
// возле ворот (cutout 80x25 с R15 угловыми кривыми). Включает обе нижние
// особые точки {52,57} и {130,57}.
std::vector<Vec> keeper_zone_points();

// Параллельный массив режимов торможения для keeper_zone_points(): индекс
// совпадает со стартовой точкой сегмента (i → ребро points[i]→points[(i+1)%N]).
// BrakeMode::Off означает «не применять Segment::apply на этом ребре»
// (виртуальная граница зоны, без физической стенки). Оба getter'а проецируются
// из общего источника истины в zones.cpp — рассинхронизация невозможна по
// конструкции.
std::vector<BrakeMode> keeper_zone_brake_modes();

// Точки физической зоны нападающего: верхняя часть поля (выше keeper-зоны).
// Нижняя грань поднята до KEEPER_ZONE_TOTAL_HEIGHT + safety, чтобы нападающий
// не мог заехать в зону вратаря (Variant A анти-коллизии).
std::vector<Vec> attacker_zone_points();

// Параллельный массив режимов торможения для attacker_zone_points(). Верхняя
// граница выреза зоны вратаря помечена BrakeMode::Low — ослабленное торможение,
// чтобы нападающий не отставал от врага в погоне; остальные рёбра Normal.
std::vector<BrakeMode> attacker_zone_brake_modes();

// Точки «зоны ответственности вратаря» — простой прямоугольник, охватывающий
// keeper-зону + ворота с запасом. Используется нападающим, чтобы решить
// «мяч у вратаря — отъезжаю». Все точки keeper_zone лежат строго внутри
// (с запасом ATTACKER_BOTTOM_SAFETY=2см), так что гистерезис hyst_inside
// не флипает на крайних точках.
std::vector<Vec> keeper_responsibility_points();

// Точки полной игровой разметки: прямоугольник x∈[12,170], y∈[12,231]
// (отступ 12 от бортов поля 182×243) с вырезами 80×25 R15 у обоих ворот.
std::vector<Vec> full_field_markup_points();

// Удобные обёртки, возвращающие готовые Polygon (нужны там, где требуется
// hyst_inside / inside / find_intersection).
Polygon make_keeper_zone();
Polygon make_attacker_zone();
Polygon make_keeper_responsibility();
Polygon make_full_field_markup();
