#include <stdio.h>
#include <wiringPi.h>

#include <chrono>
#include <fstream>
#include <thread>

#include "gpio/buttons.h"
#include "gpio/buzzer.h"
#include "robot.h"
#include "tracking/ball.h"
#include "tracking/camera.h"
#include "utils/config.h"

using namespace std;

int main() {
    load_config();
    Ball ball(config["tracking"]["ball"]["hsv_min"],
              config["tracking"]["ball"]["hsv_max"]);
    Camera camera(ball, config["preview"]["enabled"]);
    Robot robot;
    if (config["serial"]["enabled"]) {
        robot.init_uart();
    }
    if (config["gpio"]["enabled"]) {
        if (wiringPiSetupPinType(WPI_PIN_BCM) == -1) {
            cout << "Failed to setup wiringPi\n";
            exit(-1);
        }
        Buzzer buzzer(config["gpio"]["buzzer"]["pin"]);
        setup_buttons(&buzzer);
    }
    std::this_thread::sleep_for(chrono::seconds(1000));

    camera.start();

    int delay = 1000 / (int)config["tracking"]["fps"] / 2;
    while (true) {
        if (config["serial"]["enabled"]) {
            robot.read_from_arduino();
        }
        // ... strategy ...
        if (config["serial"]["enabled"]) {
            robot.write_to_arduino();
        }
        // delay
        if (config["preview"]["enabled"]) {
            camera.show_preview();
            if (cv::waitKey(delay) == 27) {
                break;
            }
        } else {
            this_thread::sleep_for(chrono::milliseconds(delay));
        }
    }
}