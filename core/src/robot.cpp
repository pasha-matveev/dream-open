#include "robot.h"

#include <libserial/SerialPort.h>

#include <iostream>
#include <thread>

#include "utils/config.h"

void Robot::read_from_arduino() {
    auto data = uart.read_data();
    angle = get<0>(data);
    emitter = get<1>(data);
    kicker_charged = get<2>(data);
}

void Robot::write_to_arduino() {
    uart.write_data(direction, speed, rotation, rotation_limit, dribling,
                    kicked_force);
}

void Robot::cycle() {
    cout << "Cycle method\n";
    while (true) {
        cout << "Read cycle\n";
        read_from_arduino();
        // serials wait for data while reading, so no delay needed
    }
}

void start_cycle(Robot *robot) { robot->cycle(); }

void Robot::start_arduino_reading() {
    uart.connect();

    thread th(start_cycle, this);
    th.detach();
}