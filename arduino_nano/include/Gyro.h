#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

class Gyro
{
  private:
    MPU6050 mpu;
    static uint32_t tmr;
    uint8_t fifoBuffer[45];
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    float trim(float);
  public:
    void init();
    void read();
    float relative(float);
    float angle;
};

void Gyro::init()
{
  Wire.begin();
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpu.setXAccelOffset(1518);
  mpu.setYAccelOffset(-3762);
  mpu.setZAccelOffset(1776);
  mpu.setXGyroOffset(24);
  mpu.setYGyroOffset(-2);
  mpu.setZGyroOffset(-11);
}

void Gyro::read()
{
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angle = ypr[0] * 57.295779;
    angle = trim(angle);
  }
}

float Gyro::trim(float raw_angle)
{
  while (raw_angle < 0)
    raw_angle += 360;
  while (raw_angle > 360)
    raw_angle -= 360;
  return raw_angle;
}

float Gyro::relative(float abs_angle)
{
  abs_angle = trim(abs_angle);
  float res = min(abs(angle - abs_angle), 360 - abs(angle - abs_angle));
  res *= (abs(angle - abs_angle) < 180 ? 1 : -1);
  res *= (angle < abs_angle ? 1 : -1);
  return res;
}