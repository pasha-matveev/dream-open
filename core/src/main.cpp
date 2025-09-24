#include <spdlog/spdlog.h>

#include <chrono>
#include <thread>

#include "robot.h"
#include "strategy/strategy.h"
#include "utils/config.h"

using namespace std;

int main() {
    spdlog::info("Loading config...");
    load_config();
    spdlog::info("Config loaded");

    Robot robot;
    spdlog::info("Initializing hardware...");
    robot.init_hardware();
    spdlog::info("Hardware ready");
    robot.rgb_led = true;

    Strategy strategy;

    int delay = 1000 / config["tracking"]["fps"].GetInt() / 2;
    while (true) {
        if (config["serial"]["enabled"].GetBool()) {
            robot.read_from_arduino();
        }
        // ... strategy ...
        strategy.run(robot);
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