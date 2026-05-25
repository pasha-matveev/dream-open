#include "config/strategy.h"

#include "config/strategy/attacker.h"
#include "config/strategy/ball_filter.h"
#include "config/strategy/control.h"
#include "config/strategy/dubins.h"
#include "config/strategy/keeper.h"
#include "config/strategy/kickoff.h"
#include "config/strategy/meme.h"
#include "utils/mapper.h"

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

  ball_filter = make_unique<BallFilter>(doc["ball_filter"]);

  motion = make_unique<Motion>();
  const rapidjson::Value& dmotion = doc["motion"];
  motion->max_linear_accel = dmotion["max_linear_accel"].GetDouble();
  motion->max_angular_accel = dmotion["max_angular_accel"].GetDouble();
  motion->decel_k = dmotion["decel_k"].GetDouble();
  motion->decel_k_low = dmotion["decel_k_low"].GetDouble();
  motion->wall_limit = dmotion["wall_limit"].GetDouble();
  motion->push_out_k = dmotion["push_out_k"].GetDouble();
  motion->push_out_v_min = dmotion["push_out_v_min"].GetDouble();

  safe_turn = make_unique<SafeTurn>();
  const rapidjson::Value& dsafe_turn = doc["safe_turn"];
  safe_turn->y = dsafe_turn["y"].GetDouble();
  safe_turn->angle = dsafe_turn["angle"].GetDouble();

  dribbling = make_unique<Mapper>(doc["dribbling"]);
  dribbling_slow = doc["dribbling_slow"].GetDouble();

  attacker = make_unique<Attacker>(doc["attacker"]);
  keeper = make_unique<Keeper>(doc["keeper"]);
  kickoff = make_unique<Kickoff>(doc["kickoff"]);
  dubins = make_unique<Dubins>(doc["dubins"]);
  control = make_unique<Control>(doc["control"]);
  meme = make_unique<Meme>(doc["meme"]);
}
