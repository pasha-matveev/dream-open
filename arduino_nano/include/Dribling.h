#include <Arduino.h>
#include <Servo.h>

class Dribling
{
private:
  Servo *ESC1 = new Servo;
  Servo *ESC2 = new Servo;
  double current_speed = 0, desired_speed = 0;

  unsigned long long lst_tm = 0;
  double change_k = 20.0 / 5000000.0;

public:
  void init();
  void set_speed(int);
  void run();
};

void Dribling::init()
{
  ESC1->attach(5);
  ESC2->attach(6);
  delay(1000);
  ESC1->writeMicroseconds(800);
  ESC2->writeMicroseconds(800);
  delay(2000);
  ESC1->writeMicroseconds(2300);
  ESC2->writeMicroseconds(2300);
  delay(2000);
  ESC1->writeMicroseconds(1450);
  ESC2->writeMicroseconds(1450);
  delay(2000);

  set_speed(0);
}

void Dribling::set_speed(int rotation)
{
  desired_speed = (double)rotation;
}

void Dribling::run()
{
  if (current_speed != desired_speed)
  {
    if ((micros() - lst_tm) * change_k < abs(desired_speed - current_speed))
    {
      current_speed += (micros() - lst_tm) * change_k * (desired_speed - current_speed) / abs(desired_speed - current_speed);
      lst_tm = micros();
    }
    else
    {
      current_speed = desired_speed;
    }
  }

  float sp1 = map(current_speed, 0, 100, 1470, 1500);
  float sp2 = map(current_speed, 0, 100, 1466, 1545);
  ESC1->writeMicroseconds(sp1);
  ESC2->writeMicroseconds(sp2);
}