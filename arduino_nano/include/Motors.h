#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

#define MOTOR_ID 0x141

class Motors
{
private:
  MCP2515 *mcp2515 = new MCP2515(10);
  struct can_frame canMsg;
  struct can_frame requestMsg;
  long ID[3] = {0x141, 0x142, 0x143};
  float kp = 7, kd = 10;
  unsigned long long lst_tm = 0;
  float lst_err = 0;

public:
  void init();
  void setpowers(int32_t *);
  void run(float, float, float, float = -1);
  void stop();
  void requestVoltageData();
  float parseMotorVoltage();
  float speed_limit = 200;
};
