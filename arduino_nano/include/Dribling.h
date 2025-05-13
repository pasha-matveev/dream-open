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
  ESC1->attach(6);
  ESC2->attach(5);
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
  if (lst_tm + 100 < millis())
  {
    if (current_speed < desired_speed)
      current_speed += 5;
    else if (current_speed > desired_speed)
      current_speed -= 5;
    lst_tm = millis();
  }

  float sp1, sp2;
  if (ROBOT)
  {
    sp1 = current_speed ? map(current_speed, 0, 100, 1466, 1545) : 1450;
    sp2 = current_speed ? map(current_speed, 0, 100, 1470, 1500) : 1450;
  }
  else
  {
    sp1 = current_speed ? map(current_speed, 0, 100, 1457, 1500) : 1450;
    sp2 = current_speed ? map(current_speed, 0, 100, 1460, 1500) : 1450;
  }

  ESC1->writeMicroseconds(sp1);
  ESC2->writeMicroseconds(sp2);
}