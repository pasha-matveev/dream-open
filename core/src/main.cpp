#include <chrono>
#include <thread>

#include "robot.h"
#include "utils/config.h"
#include "utils/sleep.h"

using namespace std;

int main() {
    load_config();

    Robot robot;
    robot.init_hardware();
    robot.rgb_led = true;

    int delay = 1000 / config["tracking"]["fps"].GetInt() / 2;
    while (true) {
        if (config["serial"]["enabled"].GetBool()) {
            robot.read_from_arduino();
        }
        // ... strategy ...
        if (config["serial"]["enabled"].GetBool()) {
            robot.write_to_arduino();
        }
        // delay
        if (config["tracking"]["preview"]["enabled"].GetBool()) {
            robot.camera->show_preview();
            if (cv::waitKey(delay) == 27) {
                break;
            }
        } else {
            this_thread::sleep_for(chrono::milliseconds(delay));
        }
    }
}