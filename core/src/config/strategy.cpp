#include "config/strategy.h"

#include "utils/mapper.h"
#include "utils/switch.h"

using namespace cfg;
using std::make_unique;

Strategy::~Strategy() = default;
Strategy::Strategy(const rapidjson::Value& doc) {
  enabled = doc["enabled"].GetBool();
  role = doc["role"].GetString();
  turn_precision = doc["turn_precision"].GetDouble();

  predict = make_unique<Predict>();
  const rapidjson::Value& dpredict = doc["predict"];
  predict->enabled = dpredict["enabled"].GetBool();
  predict->alpha_xy = dpredict["alpha_xy"].GetDouble();

  ball_filter = make_unique<BallFilter>();
  const rapidjson::Value& dbf = doc["ball_filter"];
  ball_filter->enabled = dbf["enabled"].GetBool();
  ball_filter->alpha_xy = dbf["alpha_xy"].GetDouble();
  ball_filter->beta_xy = dbf["beta_xy"].GetDouble();
  ball_filter->friction_tau = dbf["friction_tau"].GetDouble();
  ball_filter->latency_ms = dbf["latency_ms"].GetDouble();
  ball_filter->max_jump = dbf["max_jump"].GetDouble();
  ball_filter->lost_timeout_ms = dbf["lost_timeout_ms"].GetDouble();

  motion = make_unique<Motion>();
  const rapidjson::Value& dmotion = doc["motion"];
  motion->max_linear_accel = dmotion["max_linear_accel"].GetDouble();
  motion->max_angular_accel = dmotion["max_angular_accel"].GetDouble();
  motion->decel_k = dmotion["decel_k"].GetDouble();
  motion->wall_limit = dmotion["wall_limit"].GetDouble();

  dribbling = make_unique<Mapper>(doc["dribbling"]);
  dribbling_slow = doc["dribbling_slow"].GetDouble();

  attacker = make_unique<Attacker>();
  const rapidjson::Value& datt = doc["attacker"];
  attacker->border = datt["border"].GetDouble();
  attacker->dubins_enabled = datt["dubins_enabled"].GetBool();

  keeper = make_unique<Keeper>();
  const rapidjson::Value& dkeeper = doc["keeper"];
  keeper->global_border = dkeeper["global_border"].GetDouble();
  keeper->dubins_border = dkeeper["dubins_border"].GetDouble();
  keeper->ram_enabled = dkeeper["ram_enabled"].GetBool();
  keeper->line = make_unique<Keeper::Line>();
  const rapidjson::Value& dline = dkeeper["line"];
  keeper->line->padding = dline["padding"].GetDouble();
  keeper->line->y = dline["y"].GetDouble();

  dubins = make_unique<Dubins>();
  const rapidjson::Value& ddubins = doc["dubins"];
  dubins->bonus = ddubins["bonus"].GetDouble();
  dubins->separate = ddubins["separate"].GetDouble();
  dubins->radius = ddubins["radius"].GetDouble();
  dubins->deep_inside = ddubins["deep_inside"].GetDouble();
  dubins->camera_target_dist = ddubins["camera_target_dist"].GetDouble();
  dubins->kick_precision = make_unique<Switch>(ddubins["kick_precision"]);
  dubins->aim_bonus = ddubins["aim_bonus"].GetDouble();
  dubins->speed = make_unique<Mapper>(ddubins["speed"]);

  control = make_unique<Control>();
  const rapidjson::Value& dcontrol = doc["control"];
  control->speed = make_unique<Mapper>(dcontrol["speed"]);

  target_left = make_unique<Target>();
  const rapidjson::Value& dtl = doc["target_left"];
  target_left->x = dtl["x"].GetDouble();
  target_left->y = dtl["y"].GetDouble();

  target_right = make_unique<Target>();
  const rapidjson::Value& dtr = doc["target_right"];
  target_right->x = dtr["x"].GetDouble();
  target_right->y = dtr["y"].GetDouble();
}
