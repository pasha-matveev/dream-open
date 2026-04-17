#pragma once

#include <optional>
#include <queue>

#include "gpio/buzzer.h"
#include "gpio/display.h"
#include "tracking/camera.h"
#include "tracking/object.h"
#include "utils/geo/vec.h"
#include "utils/lidar.h"
#include "utils/uart.h"

using namespace LibSerial;

enum class RobotState { RUNNING, PAUSE, KICKOFF_LEFT, KICKOFF_RIGHT };

struct LidarPose {
  Vec position;
  double field_angle;
};

class Robot {
 private:
  void init_camera(Object&, Object&);
  void init_buzzer();
  void init_buttons();
  void init_uart();
  void init_gyro();
  void init_display();
  void init_lidar();

  double top_angle;

  double direction = 0;
  double speed = 0;
  bool applied_goal = false;

  struct LidarHistory {
    long long time;
    double field_angle;
    Vec position;
  };
  queue<LidarHistory> lidar_history;

 public:
  Camera* camera = nullptr;
  Buzzer* buzzer = nullptr;
  UART* uart = nullptr;
  Display* display = nullptr;
  Lidar* lidar = nullptr;

  Robot() = default;
  ~Robot();

  // Координаты центра робота на поле, в сантиметрах.
  // Оси координат направлены математически
  // Наши ворота снизу, вражеские - сверху
  Vec position = {182.0 / 2, 243.0 / 2};
  Vec last_position = {182.0 / 2, 243.0 / 2};
  // Фактическое положение.
  // 0 - на ворота противника
  // P/2 - налево
  // -P/2 - направо
  double field_angle = M_PI / 2;

  double gyro_angle = 0;
  bool emitter = false;
  bool prev_emitter = false;
  int first_time = 0;
  bool kicker_charged = false;

  // Желаемая скорость от стратегии (команда).
  Vec vel{0, 0};
  // Целевой относительный угол поворота корпуса (радианы). Arduino доворачивает
  // к нему с ограничением по rotation_limit. Семантически это угол, а не
  // угловая скорость, поэтому slew-лимитер к нему не применяется.
  double rotation = 0;
  // Сглаженная slew-лимитером оценка реальной линейной скорости робота.
  // Именно она отправляется на моторы и используется для предсказания позиции.
  Vec actual_vel{0, 0};
  double rotation_limit = 100;
  int dribling = 0;
  int kicker_force = 0;
  bool rgb_led = false;

  RobotState state = RobotState::PAUSE;

  void init_hardware(Object& ball, Object& goal);

  // Возвращает измеренную позу от лидара, если свежие данные пришли.
  // Не модифицирует robot.position / robot.field_angle — это делает
  // стратегия через EMA-блендинг с предсказанием.
  std::optional<LidarPose> compute_lidar();
  // Калибрует top_angle так, что гироскоп будет выдавать текущий field_angle.
  void calibrate();
  // Калибрует top_angle так, что гироскоп будет выдавать measured_field_angle.
  void calibrate(double measured_field_angle);
  void look_forward();
  void compute_gyro_angle();

  void read_from_arduino();
  void write_to_arduino();

  void predict_position(double dt);
  // Сглаживает vel/rotation в actual_vel/actual_rotation с ограничением
  // по линейному и угловому ускорению из config.strategy.motion.
  void apply_motion_limits(double dt);
  double relative_angle(const Vec& p) const;
  Vec ball_hole_position() const;
};