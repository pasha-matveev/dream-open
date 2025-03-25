#include <Arduino.h>
#include <Servo.h>

class Dribling
{
private:
  Servo *ESC1 = new Servo;
  Servo *ESC2 = new Servo;
  float current_speed = 0, desired_speed = 0, lst_speed = 0;
  
  unsigned long long change_tm;
  int change_limit = 0;
  float change_k = 3000.0 / 20.0;

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
}

void Dribling::set_speed(int rotation)
{
  desired_speed = (float)rotation;
  lst_speed = current_speed;
  change_tm = millis();
  change_limit = abs(desired_speed - lst_speed) * change_k;
}

void Dribling::run()
{
  if (current_speed != desired_speed)
  {
    if (change_tm + change_limit < millis())
    {
      current_speed = desired_speed;
    }
    else
    {
      float k = float(millis() - change_tm) / float(change_limit);
      current_speed = lst_speed * (1 - k) + desired_speed * k;
    }
  }
  ESC1->writeMicroseconds(map(current_speed, 0, 100, 1440, 1700));
  ESC2->writeMicroseconds(map(current_speed, 0, 100, 1440, 1705));
}