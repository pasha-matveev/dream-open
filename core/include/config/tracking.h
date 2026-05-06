#pragma once

#include <rapidjson/fwd.h>

#include <memory>
#include <string>
#include <vector>

namespace cfg {

struct Tracking {
  Tracking(const rapidjson::Value&);
  ~Tracking();
  Tracking(const Tracking&) = delete;
  Tracking& operator=(const Tracking&) = delete;

  bool enabled;
  bool preview_enabled;
  int camera_id;
  int retries;
  int fps;
  int width;
  int height;
  double brightness;
  int radius;
  int disabled_radius;

  struct Center {
    int x, y;
  };
  std::unique_ptr<Center> center;

  struct Formula {
    double a, b, c;
  };
  std::unique_ptr<Formula> formula;

  struct CameraObject {
    std::vector<int> hsv_min, hsv_max;
  };
  struct Ball : CameraObject {
    bool setup;
    int min_area;
  };
  std::unique_ptr<Ball> ball;

  struct Goal {
    std::string type;
    bool setup;
    int min_area;
    std::unique_ptr<CameraObject> yellow;
    std::unique_ptr<CameraObject> blue;
  };
  std::unique_ptr<Goal> goal;
};

}  // namespace cfg