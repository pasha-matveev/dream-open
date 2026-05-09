#pragma once

#include "utils/geo/vec.h"

namespace field_dims {

inline constexpr double kWidth = 182.0;
inline constexpr double kHeight = 243.0;

inline const Vec kCenter{kWidth / 2, kHeight / 2};
inline const Vec kOwnGoal{kWidth / 2, 0.0};
inline const Vec kOppGoal{kWidth / 2, kHeight};

}  // namespace field_dims
