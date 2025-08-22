#include <stdio.h>

#include <chrono>
#include <fstream>
#include <thread>

#include "camera.h"
#include "config.h"
#include "tracking/ball.h"

using namespace std;

int main() {
    load_config();
    Ball ball(config["tracking"]["ball"]["hsv_min"], config["tracking"]["ball"]["hsv_max"]);
    Camera camera(ball, config["preview"]["enabled"]);
    camera.start();
    int delay = 1000 / VIDEO_FPS / 2;
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