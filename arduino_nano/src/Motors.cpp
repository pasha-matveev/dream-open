#include "Motors.h"

void Motors::init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    run(0, 0, 0);
    stop();
}

void Motors::setpowers(int32_t power_arr[3]) {
    for (byte i = 0; i < 3; i++) {
        power_arr[i] *= 2390;

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
        mcp2515.sendMessage(&canMsg);
    }
}

void Motors::stop() {
    for (byte i = 0; i < 3; i++) {
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
        mcp2515.sendMessage(&canMsg);
    }
}

void Motors::run(float angle, float speed, float rotation, float rot_limit) {
    rot_limit = rot_limit == -1 ? 30 : rot_limit;
    float angles[3] = {M_PI / 3.0, M_PI, M_PI * 5.0 / 3.0};
    int32_t power_arr[3];
    // float angular_speed = rotation * kp + (rotation - lst_err) /
    // float(micros() - lst_tm) * kd;
    float angular_speed = rotation * kp + (rotation - lst_err) * kd;
    lst_err = rotation;
    lst_tm = micros();
    for (int i = 0; i < 3; i++)
        power_arr[i] = constrain(-speed * sin(angles[i] - angle), -speed_limit,
                                 speed_limit) +
                       constrain(angular_speed * 6.37, -rot_limit, rot_limit);
    setpowers(power_arr);
}

void Motors::requestVoltageData() {
    // Формируем запрос напряжения (команда 0x9A)
    requestMsg.can_id = MOTOR_ID;
    requestMsg.can_dlc = 8;
    requestMsg.data[0] = 0x9A;
    requestMsg.data[1] = 0x00;
    requestMsg.data[2] = 0x00;
    requestMsg.data[3] = 0x00;
    requestMsg.data[4] = 0x00;
    requestMsg.data[5] = 0x00;
    requestMsg.data[6] = 0x00;
    requestMsg.data[7] = 0x00;

    mcp2515.sendMessage(&requestMsg);
}

float Motors::parseMotorVoltage() {
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        uint16_t voltageRaw = (canMsg.data[2] << 8) | canMsg.data[3];
        return voltageRaw * 0.01f;
    }
    return -1;
}
