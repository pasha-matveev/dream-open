#include "strategy/zones.h"

#include <array>
#include <utility>

#include "strategy/field.h"

namespace {

// Запас между keeper-зоной и attacker-зоной (см). Нужен, чтобы field.apply()
// не пытался одновременно толкать робота с двух пересекающихся границ.
// Создаёт короткоживущую dead zone y∈[63, 65], в которой ни вратарь, ни
// нападающий не могут полноценно играть мячом — мяч обычно проходит её
// быстро, поэтому принято осознанно.
constexpr double ATTACKER_BOTTOM_SAFETY = 2.0;

// Размер keeper-полигона зафиксирован на уровне типа: добавление/удаление
// точки без обновления brake-флага не скомпилируется.
constexpr std::size_t kKeeperZoneN = 16;

// Источник истины для keeper-зоны. Каждая пара — стартовая вершина ребра и
// режим торможения на этом ребре. BrakeMode::Off у трёх рёбер: левая/правая
// боковые границы зоны (x=36, x=146) и верх (y=63) — это виртуальные пределы,
// без физической стенки, и вратарь сам выбирает позицию внутри. Остальные 13
// рёбер (низ y=12, столбы ворот x=51/x=131, четверть-окружности углов ворот,
// горизонталь линии ворот y=37) — реальная физика, нужно тормозить (Normal).
const std::array<std::pair<Vec, BrakeMode>, kKeeperZoneN>& keeper_zone_spec() {
  static const std::array<std::pair<Vec, BrakeMode>, kKeeperZoneN> kSpec = {{
      // Outer rectangle x∈[36,146], y∈[12,63]: боковые и верх — без тормоза.
      {{KEEPER_ZONE_SIDE_DIST, 12.0}, BrakeMode::Off},  // 0: левая боковая
      {{KEEPER_ZONE_SIDE_DIST, KEEPER_ZONE_TOTAL_HEIGHT},
       BrakeMode::Off},  // 1: верх
      {{FIELD_WIDTH - KEEPER_ZONE_SIDE_DIST, KEEPER_ZONE_TOTAL_HEIGHT},
       BrakeMode::Off},  // 2: правая боковая
      {{FIELD_WIDTH - KEEPER_ZONE_SIDE_DIST, 12.0},
       BrakeMode::Normal},  // 3: низ правая полоска
      // Bottom cutout 80x25 with R15 inner corners — реальная разметка вокруг
      // ворот.
      {{131.0, 12.0}, BrakeMode::Normal},  // 4: правый столб ворот
      {{131.0, 22.0}, BrakeMode::Normal},  // 5: правая arc
      {{129.9, 27.7}, BrakeMode::Normal},  // 6: правая arc
      {{126.6, 32.6}, BrakeMode::Normal},  // 7: правая arc
      {{121.7, 35.9}, BrakeMode::Normal},  // 8: правая arc
      {{116.0, 37.0}, BrakeMode::Normal},  // 9: горизонталь линии ворот y=37
      {{66.0, 37.0}, BrakeMode::Normal},   // 10: левая arc
      {{60.3, 35.9}, BrakeMode::Normal},   // 11: левая arc
      {{55.4, 32.6}, BrakeMode::Normal},   // 12: левая arc
      {{52.1, 27.7}, BrakeMode::Normal},   // 13: левая arc
      {{51.0, 22.0}, BrakeMode::Normal},   // 14: левый столб ворот
      {{51.0, 12.0}, BrakeMode::Normal},   // 15: низ левая полоска
  }};
  static_assert(kKeeperZoneN == 16, "keeper zone size must match spec table");
  return kSpec;
}

// Размер attacker-полигона зафиксирован на уровне типа: добавление/удаление
// точки без обновления режима не скомпилируется.
constexpr std::size_t kAttackerZoneN = 20;

// Источник истины для attacker-зоны. Каждая пара — стартовая вершина ребра и
// режим торможения. BrakeMode::Low только у верхней горизонтали выреза зоны
// вратаря (ребро 17) — ослабленное торможение, чтобы нападающий не отставал
// в погоне за врагом; боковые рёбра выреза (16, 18) тормозят как Normal.
const std::array<std::pair<Vec, BrakeMode>, kAttackerZoneN>&
attacker_zone_spec() {
  // U-образный вырез снизу: центральная часть поднята до верха keeper-зоны,
  // а боковые полоски (x∈[12, 34] и x∈[148, 170]) остаются до y=12 — это
  // не пересекается с keeper-зоной (она занимает x∈[36, 146]) и сохраняет
  // нападающему доступ к нижним углам поля.
  constexpr double k_top = KEEPER_ZONE_TOTAL_HEIGHT + ATTACKER_BOTTOM_SAFETY;
  constexpr double k_left =
      KEEPER_ZONE_SIDE_DIST - ATTACKER_BOTTOM_SAFETY;  // 34
  constexpr double k_right =
      FIELD_WIDTH - KEEPER_ZONE_SIDE_DIST + ATTACKER_BOTTOM_SAFETY;  // 148
  static const std::array<std::pair<Vec, BrakeMode>, kAttackerZoneN> kSpec = {{
      {{12.0, 12.0}, BrakeMode::Normal},   // 0
      {{12.0, 231.0}, BrakeMode::Normal},  // 1
      // Top cutout 80x25 with R15 inner corners (вокруг вражеских ворот).
      {{51.0, 231.0}, BrakeMode::Normal},   // 2
      {{51.0, 221.0}, BrakeMode::Normal},   // 3
      {{52.1, 215.3}, BrakeMode::Normal},   // 4
      {{55.4, 210.4}, BrakeMode::Normal},   // 5
      {{60.3, 207.1}, BrakeMode::Normal},   // 6
      {{66.0, 206.0}, BrakeMode::Normal},   // 7
      {{116.0, 206.0}, BrakeMode::Normal},  // 8
      {{121.7, 207.1}, BrakeMode::Normal},  // 9
      {{126.6, 210.4}, BrakeMode::Normal},  // 10
      {{129.9, 215.3}, BrakeMode::Normal},  // 11
      {{131.0, 221.0}, BrakeMode::Normal},  // 12
      {{131.0, 231.0}, BrakeMode::Normal},  // 13
      {{170.0, 231.0}, BrakeMode::Normal},  // 14
      {{170.0, 12.0}, BrakeMode::Normal},   // 15: ребро по y=12 — стенка поля
      // Bottom keeper-exclusion: bounding rectangle of keeper-зоны + safety.
      {{k_right, 12.0}, BrakeMode::Normal},  // 16: правая стенка выреза
      {{k_right, k_top}, BrakeMode::Low},    // 17: верх выреза — ослабленно
      {{k_left, k_top}, BrakeMode::Normal},  // 18: левая стенка выреза
      {{k_left, 12.0}, BrakeMode::Normal},   // 19: ребро по y=12 — стенка поля
  }};
  static_assert(kAttackerZoneN == 20,
                "attacker zone size must match spec table");
  return kSpec;
}

}  // namespace

std::vector<Vec> keeper_zone_points() {
  const auto& spec = keeper_zone_spec();
  std::vector<Vec> points;
  points.reserve(spec.size());
  for (const auto& [pt, brake] : spec) {
    points.push_back(pt);
  }
  return points;
}

std::vector<BrakeMode> keeper_zone_brake_modes() {
  const auto& spec = keeper_zone_spec();
  std::vector<BrakeMode> modes;
  modes.reserve(spec.size());
  for (const auto& [pt, mode] : spec) {
    modes.push_back(mode);
  }
  return modes;
}

std::vector<Vec> attacker_zone_points() {
  const auto& spec = attacker_zone_spec();
  std::vector<Vec> points;
  points.reserve(spec.size());
  for (const auto& [pt, mode] : spec) {
    points.push_back(pt);
  }
  return points;
}

std::vector<BrakeMode> attacker_zone_brake_modes() {
  const auto& spec = attacker_zone_spec();
  std::vector<BrakeMode> modes;
  modes.reserve(spec.size());
  for (const auto& [pt, mode] : spec) {
    modes.push_back(mode);
  }
  return modes;
}

std::vector<Vec> keeper_responsibility_points() {
  // Простой прямоугольник: bounding box keeper-зоны, расширенный на
  // ATTACKER_BOTTOM_SAFETY (2 см) с каждой стороны. Гарантированно
  // содержит все точки keeper_zone, ATTACKER_BOTTOM_SAFETY запас
  // компенсирует гистерезис hyst_inside (±2 см).
  const double left = KEEPER_ZONE_SIDE_DIST - ATTACKER_BOTTOM_SAFETY;  // 34
  const double right =
      FIELD_WIDTH - KEEPER_ZONE_SIDE_DIST + ATTACKER_BOTTOM_SAFETY;      // 148
  const double top = KEEPER_ZONE_TOTAL_HEIGHT + ATTACKER_BOTTOM_SAFETY;  // 65
  return {
      {left, 0.0},
      {left, top},
      {right, top},
      {right, 0.0},
  };
}

std::vector<Vec> full_field_markup_points() {
  // Простой многоугольник (single closed curve, без отдельных hole-loop'ов):
  // внешний прямоугольник 12..170 × 12..231 с двумя «прокусами» под ворота.
  // Точки выреза скопированы как есть из attacker_zone (верх) и keeper_zone
  // (низ) — порядок обхода уже согласован с их соответствующими сторонами,
  // здесь просто склеиваются в общем CW-обходе.
  return {
      {12.0, 12.0},
      {12.0, 231.0},
      // Top cutout 80×25 R15 (вражеские ворота) — копия из attacker_zone.
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
      // Bottom cutout 80×25 R15 (свои ворота) — копия из keeper_zone_spec.
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
      // Замыкающее ребро (51,12) → (12,12) добавит сам Polygon.
  };
}

Polygon make_keeper_zone() { return Polygon(keeper_zone_points()); }
Polygon make_attacker_zone() { return Polygon(attacker_zone_points()); }
Polygon make_keeper_responsibility() {
  return Polygon(keeper_responsibility_points());
}
Polygon make_full_field_markup() {
  return Polygon(full_field_markup_points());
}
