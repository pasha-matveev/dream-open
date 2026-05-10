#include "config/strategy/control.h"

using namespace cfg;
using std::make_unique;

Control::~Control() = default;

Control::Control(const rapidjson::Value& doc) {
  speed = make_unique<Mapper>(doc["speed"]);
  kick_power = make_unique<Mapper>(doc["kick_power"]);

  curved_turn = make_unique<CurveTurn>();
  const rapidjson::Value& dct = doc["curved_turn"];
  curved_turn->accel_time = dct["accel_time"].GetInt();
  curved_turn->rotation_limit = dct["rotation_limit"].GetDouble();
  curved_turn->vel = dct["vel"].GetDouble();
  curved_turn->dist = dct["dist"].GetDouble();

  simple_turn = make_unique<SimpleTurn>();
  const rapidjson::Value& dst = doc["simple_turn"];
  simple_turn->accel_time = dst["accel_time"].GetInt();
  simple_turn->rotation_limit = dst["rotation_limit"].GetInt();

  ball_drive = make_unique<BallDrive>();
  const rapidjson::Value& dbd = doc["ball_drive"];
  ball_drive->max_accel = dbd["max_accel"].GetDouble();
}