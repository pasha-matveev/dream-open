#pragma once

#include <rapidjson/fwd.h>

#include <memory>

struct Mapper;

namespace cfg {

struct Kickoff {
  Kickoff(const rapidjson::Value& doc);
  ~Kickoff();
  Kickoff(const Kickoff&) = delete;
  Kickoff& operator=(const Kickoff&) = delete;

  // Скорость дриблинга
  std::unique_ptr<Mapper> dribbling;

  // Максимум "слепого" хода вперёд для захвата мяча (мс).
  long long capture_blind_timeout_ms;
  // Начальная скорость в фазе CAPTURE_BLIND (см/с). Линейно падает к 0
  // к концу таймаута.
  double capture_speed;

  struct Target {
    double x, y;
  };
  // Точка на поле, в которую хотим попасть мячом ПОСЛЕ рикошета от боковой
  // стенки. target_left используется при левой разводке (рикошет от левой
  // стенки x=0), target_right — при правой (от правой стенки x=182).
  Target target_left;
  Target target_right;

  // Сила удара кикером.
  int kick_force;
  // Сколько держать kicker_force перед переходом в DONE (мс).
  long long kick_followthrough_ms;
  // Пауза после DONE до того, как обычная роль (attacker) возьмёт управление.
  // Реализуется через Strategy::stop_until.
  long long post_kickoff_delay_ms;
};

}  // namespace cfg
