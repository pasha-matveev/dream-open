

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include "Robot.h"

Robot robot;
Adafruit_NeoPixel pixels(2, 9, NEO_GRB + NEO_KHZ800);
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
  // robot.init_kicker = false;
  // robot.init_dribling = false;
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  pixels.show();
  robot.init();
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();

  // delay(6000);
  robot.gyro.generate_correction();
  Serial.begin(115200);
}

void loop()
{
  // robot.dribling.set_speed(100);
  // robot.dribling.run();
  // Serial.println(analogRead(A0));
  // delay(1);



  // if (test_tm < millis())
  // {
  //   robot.direction += M_PI / 2;
  //   test_tm = millis() + 500;
  // }

  // robot.read();

  // robot.rotation = 0;
  // robot.speed = 50;
  // robot.rotation_limit = 30;

  // robot.run();



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