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
    char bytes[6];  // float + bool + bool = 4 + 1 + 1 = 6
    serial.read(bytes, 6);

    float angle;
    memcpy(&angle, bytes, 4);
    bool emitter = bytes[4];
    bool kicker = bytes[5];

    return {angle, emitter, kicker};
}