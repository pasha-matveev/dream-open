#include "uart.h"

#include "config.h"

void UART::connect() {
    serial.Open(config["serial"]["device"]);
    serial.SetBaudRate(config["serial"]["rate"]);
    serial.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serial.SetParity(Parity::PARITY_NONE);
    serial.SetStopBits(StopBits::STOP_BITS_1);
    serial.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
}

tuple<float, bool, bool> UART::read_data() {
    char buffer[6];  // float + bool + bool = 4 + 1 + 1 = 6
    serial.read(buffer, 6);

    float angle;
    memcpy(&angle, buffer, 4);
    bool emitter = buffer[4];
    bool kicker = buffer[5];

    return {angle, emitter, kicker};
}

void UART::write_data(float direction, float speed, float rotation,
                      float rotation_limit, int dribling, int kicked_force) {
    char buffer[24];  // float + float + float float + int + int = 4 * 6 = 24
    memcpy(buffer, &direction, 4);
    memcpy(buffer + 4, &speed, 4);
    memcpy(buffer + 8, &rotation, 4);
    memcpy(buffer + 12, &rotation_limit, 4);
    memcpy(buffer + 16, &dribling, 4);
    memcpy(buffer + 20, &kicked_force, 4);

    serial.write(buffer, 24);
}