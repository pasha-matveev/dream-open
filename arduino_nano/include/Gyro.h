#pragma once
#include <Arduino.h>
#include <MPU6050_6Axis_MotionApps20.h>

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
    void print_offsets();
    float relative(float);
    void generate_correction();
    float angle;

    float correction = 0;
};
