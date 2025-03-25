#include <Arduino.h>
#include "Robot.h"

Robot robot;
unsigned long long alive_tm;
unsigned long long test_tm;

template <typename T>
T read_data()
{
  T result;
  Serial.readBytes(reinterpret_cast<byte *>(&result), sizeof(T));
  return result;
}

template <class T>
void write_data(T var)
{
  Serial.write((const byte *)&var, sizeof(var));
}

void setup()
{
  Serial.begin(115200);
  robot.init_dribling = false;
  robot.init_kicker = false;
  robot.init();
}

void loop()
{
  if (Serial.available())
  {
    robot.direction = read_data<float>();
    robot.speed = read_data<float>();
    robot.rotation = read_data<float>();
    robot.dribling->set_speed(read_data<int32_t>());
    robot.kicker->force = read_data<int32_t>();

    write_data(robot.gyro->angle);
    write_data(robot.emitter->val);
    write_data(robot.kicker->is_ready());

    robot.dribling->set_speed(robot.dribling_speed);
    robot.kicker->kick(robot.kicker_force);

    alive_tm = millis() + 1000;
  }

  if (alive_tm > millis())
    robot.run();
  else
    robot.stop();
}