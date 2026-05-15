#include "config/strategy/keeper.h"

#include "utils/mapper.h"

using namespace cfg;
using std::make_unique;

Keeper::~Keeper() = default;

Keeper::Keeper(const rapidjson::Value& doc) {
  projection_border = doc["projection_border"].GetDouble();

  line = make_unique<Line>();
  const rapidjson::Value& dline = doc["line"];
  line->padding = dline["padding"].GetDouble();
  line->y = dline["y"].GetDouble();
  line->ray_min_y = dline["ray_min_y"].GetDouble();

  push_out = make_unique<PushOut>();
  const rapidjson::Value& dpush_out = doc["push_out"];
  push_out->k = dpush_out["k"].GetDouble();
  push_out->v_min = dpush_out["v_min"].GetDouble();

  ram = make_unique<Ram>();
  const rapidjson::Value& dram = doc["ram"];
  ram->enabled = dram["enabled"].GetBool();
  ram->safety_y = dram["safety_y"].GetDouble();
  ram->far_min_dist = dram["far_min_dist"].GetDouble();
  ram->blind_min_ms = dram["blind_min_ms"].GetInt64();
  ram->max_speed = dram["max_speed"].GetDouble();
  ram->min_speed = dram["min_speed"].GetDouble();

  additional_responsibility = make_unique<AdditionalResponsibility>();
  const rapidjson::Value& dar = doc["additional_responsibility"];
  additional_responsibility->y_max = dar["y_max"].GetDouble();
  additional_responsibility->x_padding = dar["x_padding"].GetDouble();

  ricochet = make_unique<Ricochet>();
  const rapidjson::Value& drc = doc["ricochet"];
  ricochet->enabled = drc["enabled"].GetBool();
  ricochet->x_padding = drc["x_padding"].GetDouble();
  ricochet->hysteresis = drc["hysteresis"].GetDouble();
  ricochet->recompute_angle_each_tick =
      drc["recompute_angle_each_tick"].GetBool();
  ricochet->target_left.x = drc["target_left"]["x"].GetDouble();
  ricochet->target_left.y = drc["target_left"]["y"].GetDouble();
  ricochet->target_right.x = drc["target_right"]["x"].GetDouble();
  ricochet->target_right.y = drc["target_right"]["y"].GetDouble();
  ricochet->dribbling_slowdown =
      make_unique<Mapper>(drc["dribbling_slowdown"]);
}
