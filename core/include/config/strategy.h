#pragma once

#include <rapidjson/fwd.h>

#include <memory>
#include <string>

struct Mapper;

namespace cfg {

struct Control;
struct BallFilter;
struct Attacker;
struct Keeper;
struct Kickoff;
struct Dubins;

struct Strategy {
  Strategy(const rapidjson::Value&);
  ~Strategy();
  Strategy(const Strategy&) = delete;
  Strategy& operator=(const Strategy&) = delete;

  bool enabled;
  std::string role;
  double turn_precision;

  struct Predict {
    bool enabled;
    double alpha_xy;
  };
  std::unique_ptr<Predict> predict;

  std::unique_ptr<BallFilter> ball_filter;

  struct Motion {
    double max_linear_accel;
    double max_angular_accel;
    double decel_k;
    // Ослабленный decel_k для рёбер с BrakeMode::Low. Должен быть <= 1.0:
    // при 1.0 робот тормозит ровно на физическом пределе, при значении
    // больше 1.0 не успевает остановиться и заезжает за границу.
    double decel_k_low;
    double wall_limit;
    double push_out_k;
    double push_out_v_min;
  };
  std::unique_ptr<Motion> motion;

  struct SafeTurn {
    double y;      // ниже этого y (см) включается безопасный разворот
    double angle;  // порог |доворота| (рад), выше которого крутим через стенку
  };
  std::unique_ptr<SafeTurn> safe_turn;

  std::unique_ptr<Mapper> dribbling;
  double dribbling_slow;

  std::unique_ptr<Attacker> attacker;
  std::unique_ptr<Keeper> keeper;
  std::unique_ptr<Kickoff> kickoff;
  std::unique_ptr<Dubins> dubins;
  std::unique_ptr<Control> control;
};

}  // namespace cfg
