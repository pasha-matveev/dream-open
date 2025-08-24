#include <stdio.h>

#include <chrono>
#include <fstream>
#include <thread>

#include "camera.h"
#include "config.h"
#include "robot.h"
#include "tracking/ball.h"

using namespace std;

int main() {
    load_config();
    Ball ball(config["tracking"]["ball"]["hsv_min"],
              config["tracking"]["ball"]["hsv_max"]);
    Camera camera(ball, config["preview"]["enabled"]);
    Robot robot;

    camera.start();
    robot.start_arduino_reading();

    int delay = 1000 / (int)config["tracking"]["fps"] / 2;
    while (true) {
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