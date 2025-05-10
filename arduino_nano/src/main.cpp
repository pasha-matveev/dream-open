#define ROBOT 0

#include <Arduino.h>
#include "Robot.h"

Robot robot;
unsigned long long alive_tm;
unsigned long long test_tm = 10000;


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
  robot.read();

  if (Serial.available())
  {
    robot.direction = read_data<float>();
    robot.speed = read_data<float>();
    robot.rotation = read_data<float>();
    robot.dribling.set_speed(read_data<int32_t>());
    robot.kicker.force = read_data<int32_t>();

    write_data(robot.direction);
    write_data(robot.speed);
    write_data(robot.rotation);

    write_data(robot.gyro.angle);
    write_data((millis() - robot.emitter.last_tm) < 500);
    write_data(robot.kicker.is_ready());

    robot.kicker.kick(robot.kicker.force);

    alive_tm = millis() + 1000;
  }

  if (alive_tm > millis())
    robot.run();
  else
    robot.stop();
}