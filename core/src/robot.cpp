#include "robot.h"

#include <libserial/SerialPort.h>

#include <iostream>
#include <thread>

#include "utils/config.h"

void Robot::read_from_arduino() {
    uart.write_data<char>('R');
    angle = uart.read_data<float>();
    emitter = uart.read_data<bool>();
    kicker_charged = uart.read_data<bool>();
}

void Robot::write_to_arduino() {
    uart.write_data<char>('W');
    uart.write_data<float>(direction);
    uart.write_data<float>(speed);
    uart.write_data<float>(rotation);
    uart.write_data<float>(rotation_limit);
    uart.write_data<int32_t>(dribling);
    uart.write_data<int32_t>(kicker_force);
}

void Robot::init_uart() {
    uart.connect();
    uart.wait_for_x();
}