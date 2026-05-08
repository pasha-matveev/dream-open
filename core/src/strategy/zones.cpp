#include "strategy/zones.h"

#include "strategy/field.h"

namespace {

// Запас между keeper-зоной и attacker-зоной (см). Нужен, чтобы field.apply()
// не пытался одновременно толкать робота с двух пересекающихся границ.
// Создаёт короткоживущую dead zone y∈[63, 65], в которой ни вратарь, ни
// нападающий не могут полноценно играть мячом — мяч обычно проходит её
// быстро, поэтому принято осознанно.
constexpr double ATTACKER_BOTTOM_SAFETY = 2.0;

}  // namespace

std::vector<Vec> keeper_zone_points() {
  return {
      // Outer rectangle: x∈[36,146], y∈[12,63].
      {KEEPER_ZONE_SIDE_DIST, 12.0},
      {KEEPER_ZONE_SIDE_DIST, KEEPER_ZONE_TOTAL_HEIGHT},
      {FIELD_WIDTH - KEEPER_ZONE_SIDE_DIST, KEEPER_ZONE_TOTAL_HEIGHT},
      {FIELD_WIDTH - KEEPER_ZONE_SIDE_DIST, 12.0},
      // Bottom cutout 80x25 with R15 inner corners (где физически ворота —
      // робот туда заехать не может).
      {131.0, 12.0},
      {131.0, 22.0},
      {129.9, 27.7},
      {126.6, 32.6},
      {121.7, 35.9},
      {116.0, 37.0},
      {66.0, 37.0},
      {60.3, 35.9},
      {55.4, 32.6},
      {52.1, 27.7},
      {51.0, 22.0},
      {51.0, 12.0},
  };
}

std::vector<Vec> attacker_zone_points() {
  // U-образный вырез снизу: центральная часть поднята до верха keeper-зоны,
  // а боковые полоски (x∈[12, 34] и x∈[148, 170]) остаются до y=12 — это
  // не пересекается с keeper-зоной (она занимает x∈[36, 146]) и сохраняет
  // нападающему доступ к нижним углам поля.
  const double k_top = KEEPER_ZONE_TOTAL_HEIGHT + ATTACKER_BOTTOM_SAFETY;
  const double k_left = KEEPER_ZONE_SIDE_DIST - ATTACKER_BOTTOM_SAFETY;            // 34
  const double k_right = FIELD_WIDTH - KEEPER_ZONE_SIDE_DIST + ATTACKER_BOTTOM_SAFETY;  // 148
  return {
      {12.0, 12.0},
      {12.0, 231.0},
      // Top cutout 80x25 with R15 inner corners (вокруг вражеских ворот).
      {51.0, 231.0},
      {51.0, 221.0},
      {52.1, 215.3},
      {55.4, 210.4},
      {60.3, 207.1},
      {66.0, 206.0},
      {116.0, 206.0},
      {121.7, 207.1},
      {126.6, 210.4},
      {129.9, 215.3},
      {131.0, 221.0},
      {131.0, 231.0},
      {170.0, 231.0},
      {170.0, 12.0},
      // Bottom keeper-exclusion: bounding rectangle of keeper-зоны + safety.
      // Дополнительно покрывает все крылья и goal cutout вратаря — нападающий
      // в эту зону не заезжает, столкновений быть не должно.
      {k_right, 12.0},
      {k_right, k_top},
      {k_left, k_top},
      {k_left, 12.0},
  };
}

Polygon make_keeper_zone() { return Polygon(keeper_zone_points()); }
Polygon make_attacker_zone() { return Polygon(attacker_zone_points()); }
