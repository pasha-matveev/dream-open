#pragma once

#include <limits>
#include <optional>

#include "utils/geo/vec.h"

class Robot;

// Ближайшее к ref_point препятствие по данным лидара (поле-координаты),
// либо nullopt. Препятствия с y < min_y игнорируются (зона нашего вратаря —
// лидар не отличает вражеского робота от своего).
// Self-фильтр и фильтр по разметке поля — config->lidar->piter.
std::optional<Vec> nearest_obstacle(
    Robot& robot, const Vec& ref_point,
    double min_y = -std::numeric_limits<double>::infinity());
