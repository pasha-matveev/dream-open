#include <Arduino.h>
#include "SPI.h"
#include "mcp2515.h"

class Motors
{
private:
  MCP2515 *mcp2515 = new MCP2515(10);
  struct can_frame canMsg;
  long ID[3] = {0x141, 0x142, 0x143};
  float kp = 2, kd = 10000;
  unsigned long long lst_tm = 0;
  float lst_err = 0;

public:
  void init();
  void setpowers(int32_t *);
  void run(float, float, float);
  void stop();
  float speed_limit = 100;
  float rotation_limit = 40;
};

void Motors::init()
{
  mcp2515->reset();
  mcp2515->setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515->setNormalMode();
}

void Motors::setpowers(int32_t power_arr[3])
{
  for (byte i = 0; i < 3; i++)
  {
    power_arr[i] *= 900;

    canMsg.can_id = ID[i];
    canMsg.can_dlc = 8;
    canMsg.data[0] = 0xA2;
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = *(uint8_t *)(&power_arr[i]);
    canMsg.data[5] = *((uint8_t *)(&power_arr[i]) + 1);
    canMsg.data[6] = *((uint8_t *)(&power_arr[i]) + 2);
    canMsg.data[7] = *((uint8_t *)(&power_arr[i]) + 3);
    mcp2515->sendMessage(&canMsg);
  }
}

void Motors::stop()
{
  for (byte i = 0; i < 3; i++)
  {
    canMsg.can_id = ID[i];
    canMsg.can_dlc = 8;
    canMsg.data[0] = 0x81;
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
    mcp2515->sendMessage(&canMsg);
  }
}

void Motors::run(float angle, float speed, float rotation)
{
  int angles[3] = {60, 180, 300};
  int32_t power_arr[3];
  float angular_speed = rotation * kp + (rotation - lst_err) / float(micros() - lst_tm) * kd;
  lst_err = rotation;
  lst_tm = micros();
  for (int i = 0; i < 3; i++)
    power_arr[i] = constrain(-speed * sin(radians(angles[i] - angle)), -speed_limit, speed_limit) +
                   constrain(angular_speed, -rotation_limit, rotation_limit);
  setpowers(power_arr);
}
