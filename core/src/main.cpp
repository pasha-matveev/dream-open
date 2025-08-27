#include <chrono>
#include <thread>

#include "robot.h"
#include "utils/config.h"

using namespace std;

int main() {
    load_config();

    Robot robot;
    robot.init_hardware();

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
        if (config["tracking"]["preview"]["enabled"]) {
            robot.camera->show_preview();
            if (cv::waitKey(delay) == 27) {
                break;
            }
        } else {
            this_thread::sleep_for(chrono::milliseconds(delay));
        }
    }
}