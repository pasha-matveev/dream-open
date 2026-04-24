#include "utils/config.h"

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

#include <fstream>
#include <stdexcept>

using namespace std;

using rapidjson::Document;
Config config;

template <typename Array>
vector<int> to_int_vector(const Array& arr) {
  vector<int> result;
  result.reserve(arr.Size());
  for (const auto& value : arr) {
    result.push_back(value.GetInt());
  }
  return result;
}

void load_config() {
  ifstream file("config.json");
  if (!file.is_open()) {
    throw runtime_error("Unable to open config.json");
  }

  rapidjson::IStreamWrapper isw(file);
  Document doc;
  doc.ParseStream(isw);

  if (doc.HasParseError()) {
    throw runtime_error("Failed to parse config.json");
  }

  Config loaded;

  const auto& tracking = doc["tracking"];
  loaded.tracking.enabled = tracking["enabled"].GetBool();
  const auto& preview = tracking["preview"];
  loaded.tracking.preview.enabled = preview["enabled"].GetBool();
  loaded.tracking.preview.window_name = preview["window_name"].GetString();
  loaded.tracking.camera_id = tracking["camera_id"].GetInt();
  loaded.tracking.retries = tracking["retries"].GetInt();
  loaded.tracking.fps = tracking["fps"].GetInt();
  loaded.tracking.width = tracking["width"].GetInt();
  loaded.tracking.height = tracking["height"].GetInt();
  loaded.tracking.brightness = tracking["brightness"].GetDouble();
  loaded.tracking.radius = tracking["radius"].GetInt();
  loaded.tracking.disabled_radius = tracking["disabled_radius"].GetInt();
  const auto& center = tracking["center"];
  loaded.tracking.center.x = center["x"].GetInt();
  loaded.tracking.center.y = center["y"].GetInt();
  const auto& formula = tracking["formula"];
  loaded.tracking.formula.a = formula["a"].GetDouble();
  loaded.tracking.formula.b = formula["b"].GetDouble();
  loaded.tracking.formula.c = formula["c"].GetDouble();
  const auto& ball = tracking["ball"];
  loaded.tracking.ball.setup = ball["setup"].GetBool();
  loaded.tracking.ball.min_area = ball["min_area"].GetInt();
  loaded.tracking.ball.hsv_min = to_int_vector(ball["hsv_min"].GetArray());
  loaded.tracking.ball.hsv_max = to_int_vector(ball["hsv_max"].GetArray());

  const auto& goal = tracking["goal"];
  loaded.tracking.goal.setup = goal["setup"].GetBool();
  loaded.tracking.goal.type = goal["type"].GetString();
  loaded.tracking.goal.min_area = goal["min_area"].GetInt();

  const auto& yellow = goal["yellow"];
  loaded.tracking.goal.yellow.hsv_min =
      to_int_vector(yellow["hsv_min"].GetArray());
  loaded.tracking.goal.yellow.hsv_max =
      to_int_vector(yellow["hsv_max"].GetArray());

  const auto& blue = goal["blue"];
  loaded.tracking.goal.blue.hsv_min = to_int_vector(blue["hsv_min"].GetArray());
  loaded.tracking.goal.blue.hsv_max = to_int_vector(blue["hsv_max"].GetArray());

  const auto& serial = doc["serial"];
  loaded.serial.enabled = serial["enabled"].GetBool();
  loaded.serial.device = serial["device"].GetString();
  loaded.serial.rate = serial["rate"].GetInt();
  loaded.serial.interference = serial["interference"].GetBool();

  const auto& gpio = doc["gpio"];
  loaded.gpio.enabled = gpio["enabled"].GetBool();
  const auto& buzzer = gpio["buzzer"];
  loaded.gpio.buzzer.enabled = buzzer["enabled"].GetBool();
  loaded.gpio.buzzer.pin = buzzer["pin"].GetInt();
  loaded.gpio.buzzer.notes = buzzer["notes"].GetInt();
  const auto& buttons = gpio["buttons"];
  loaded.gpio.buttons.enabled = buttons["enabled"].GetBool();
  loaded.gpio.buttons.pins = to_int_vector(buttons["pins"].GetArray());
  const auto& display = gpio["display"];
  loaded.gpio.display.enabled = display["enabled"].GetBool();
  loaded.gpio.display.device = display["device"].GetString();
  loaded.gpio.display.address = display["address"].GetString();
  loaded.gpio.display.mode = display["mode"].GetString();
  loaded.gpio.display.img = display["img"].GetInt();

  const auto& lidar = doc["lidar"];
  loaded.lidar.enabled = lidar["enabled"].GetBool();
  loaded.lidar.path = lidar["path"].GetString();
  const auto& calibration = lidar["calibration"];
  loaded.lidar.calibration.enabled = calibration["enabled"].GetBool();
  loaded.lidar.calibration.delay = calibration["delay"].GetInt();
  loaded.lidar.calibration.threshold = calibration["threshold"].GetInt();
  loaded.lidar.calibration.movement = calibration["movement"].GetDouble();
  loaded.lidar.calibration.angle = calibration["angle"].GetDouble();

  const auto& visualization = doc["visualization"];
  loaded.visualization.enabled = visualization["enabled"].GetBool();
  loaded.visualization.window_name = visualization["window_name"].GetString();
  loaded.visualization.frames = visualization["frames"].GetInt();
  loaded.visualization.interactive = visualization["interactive"].GetBool();

  const auto& strategy = doc["strategy"];
  loaded.strategy.enabled = strategy["enabled"].GetBool();
  loaded.strategy.role = strategy["role"].GetString();
  loaded.strategy.turn_precision = strategy["turn_precision"].GetDouble();
  const auto& predict = strategy["predict"];
  loaded.strategy.predict.enabled = predict["enabled"].GetBool();
  loaded.strategy.predict.alpha_xy = predict["alpha_xy"].GetDouble();

  const auto& ball_filter = strategy["ball_filter"];
  loaded.strategy.ball_filter.alpha_xy = ball_filter["alpha_xy"].GetDouble();
  loaded.strategy.ball_filter.beta_xy = ball_filter["beta_xy"].GetDouble();
  loaded.strategy.ball_filter.friction_tau =
      ball_filter["friction_tau"].GetDouble();
  loaded.strategy.ball_filter.latency_ms =
      ball_filter["latency_ms"].GetDouble();
  loaded.strategy.ball_filter.max_jump = ball_filter["max_jump"].GetDouble();
  loaded.strategy.ball_filter.lost_timeout_ms =
      ball_filter["lost_timeout_ms"].GetDouble();
  loaded.strategy.dribbling = Mapper(strategy["dribbling"]);
  loaded.strategy.dribbling_slow = strategy["dribbling_slow"].GetDouble();

  const auto& motion = strategy["motion"];
  loaded.strategy.motion.max_linear_accel =
      motion["max_linear_accel"].GetDouble();
  loaded.strategy.motion.max_angular_accel =
      motion["max_angular_accel"].GetDouble();
  loaded.strategy.motion.decel_k = motion["decel_k"].GetDouble();
  loaded.strategy.motion.wall_limit = motion["wall_limit"].GetDouble();

  const auto& attacker = strategy["attacker"];
  loaded.strategy.attacker.border = attacker["border"].GetDouble();
  loaded.strategy.attacker.dubins_enabled =
      attacker["dubins_enabled"].GetBool();

  const auto& keeper = strategy["keeper"];
  loaded.strategy.keeper.global_border = keeper["global_border"].GetDouble();
  loaded.strategy.keeper.dubins_border = keeper["dubins_border"].GetDouble();
  loaded.strategy.keeper.ram_enabled = keeper["ram_enabled"].GetBool();
  const auto& keeper_line = keeper["line"];
  loaded.strategy.keeper.line.padding = keeper_line["padding"].GetDouble();
  loaded.strategy.keeper.line.y = keeper_line["y"].GetDouble();

  const auto& dubins = strategy["dubins"];
  loaded.strategy.dubins.radius = dubins["radius"].GetDouble();
  loaded.strategy.dubins.deep_inside = dubins["deep_inside"].GetDouble();
  loaded.strategy.dubins.bonus = dubins["bonus"].GetDouble();
  loaded.strategy.dubins.separate = dubins["separate"].GetDouble();
  loaded.strategy.dubins.camera_target_dist =
      dubins["camera_target_dist"].GetDouble();
  loaded.strategy.dubins.kick_precision = Switch{dubins["kick_precision"]};
  loaded.strategy.dubins.speed = Mapper(dubins["speed"]);

  const auto& target_left = strategy["target_left"];
  loaded.strategy.target_left.x = target_left["x"].GetDouble();
  loaded.strategy.target_left.y = target_left["y"].GetDouble();
  const auto& target_right = strategy["target_right"];
  loaded.strategy.target_right.x = target_right["x"].GetDouble();
  loaded.strategy.target_right.y = target_right["y"].GetDouble();

  config = loaded;
}
