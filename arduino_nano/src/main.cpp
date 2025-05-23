#define ROBOT 0 // 0 - black robot, 1 - green robot

#include <Arduino.h>
#include "Robot.h"

Robot robot;
unsigned long long alive_tm;

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
  // robot.init_kicker = false;
  // robot.init_dribling = false;
  robot.init();
  // delay(6000);
  robot.gyro.generate_correction();
  Serial.begin(115200);
}

void loop()
{
  // robot.dribling.set_speed(60);
  // robot.dribling.run();
  // Serial.println(analogRead(A0));
  // delay(1);

  robot.read();

  if (Serial.available())
  {
    robot.direction = read_data<float>();
    robot.speed = read_data<float>();
    robot.rotation = read_data<float>();
    robot.rotation_limit = read_data<float>();
    robot.dribling.set_speed(read_data<int32_t>());
    robot.kicker_force = read_data<int32_t>();

    if (millis() - robot.emitter.first_tm > 100)
    {
      robot.emitter.reset();
      robot.kicker.set_force(robot.kicker_force);
    }

    write_data(robot.gyro.angle);
    write_data((millis() - robot.emitter.last_tm) < 500);
    write_data(robot.kicker.is_charged());

    alive_tm = millis() + 1000;
  }

  if (alive_tm > millis() && robot.init_kicker)
  {
    robot.kicker.charge();
    if (robot.button.state())
      robot.kicker.kick();
  }
  else
  {
    robot.kicker.charge(false);
  }

  if (alive_tm > millis() && robot.button.state())
    robot.run();
  else
    robot.stop();
}