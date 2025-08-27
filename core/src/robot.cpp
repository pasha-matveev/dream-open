#include "robot.h"

#include <libserial/SerialPort.h>
#include <wiringPi.h>

#include <iostream>
#include <thread>

#include "gpio/buttons.h"
#include "gpio/buzzer.h"
#include "utils/config.h"

void Robot::read_from_arduino() {
    uart->write_data<char>('R');
    angle = uart->read_data<float>();
    emitter = uart->read_data<bool>();
    kicker_charged = uart->read_data<bool>();
}

void Robot::write_to_arduino() {
    uart->write_data<char>('W');
    uart->write_data<float>(direction);
    uart->write_data<float>(speed);
    uart->write_data<float>(rotation);
    uart->write_data<float>(rotation_limit);
    uart->write_data<int32_t>(dribling);
    uart->write_data<int32_t>(kicker_force);
}

void Robot::init_camera() {
    auto v1 = config["tracking"]["ball"]["hsv_min"].GetArray();
    auto v2 = config["tracking"]["ball"]["hsv_max"].GetArray();
    Ball ball(make_int_vector(v1), make_int_vector(v2));
    camera =
        new Camera(ball, config["tracking"]["preview"]["enabled"].GetBool());
    camera->start();
}

void Robot::init_buzzer() {
    buzzer = new Buzzer(config["gpio"]["buzzer"]["pin"].GetInt());
}

void Robot::init_buttons() { setup_buttons(buzzer); }

void Robot::init_uart() {
    uart = new UART();
    uart->connect();
    uart->wait_for_x();
}

void Robot::init_hardware() {
    if (config["tracking"]["enabled"].GetBool()) {
        init_camera();
    }
    if (config["gpio"]["enabled"].GetBool()) {
        if (wiringPiSetupPinType(WPI_PIN_BCM) == -1) {
            cout << "Failed to setup wiringPi\n";
            exit(-1);
        }
        init_buzzer();
        init_buttons();
    }
    if (config["serial"]["enabled"].GetBool()) {
        init_uart();
    }
}