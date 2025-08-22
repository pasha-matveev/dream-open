#include <stdio.h>

#include <chrono>
#include <fstream>
#include <thread>

#include "camera.h"
#include "json.hpp"
#include "tracking/ball.h"

using namespace std;
using nlohmann::json;

int main() {
    ifstream config_file("./config.json");
    json config = json::parse(config_file);
    config_file.close();

    Ball ball;
    Camera camera(ball, config["preview"]);
    camera.start();
    int delay = 1000 / VIDEO_FPS / 2;
    while (true) {
        if (config["preview"]) {
            camera.show_preview();
            if (cv::waitKey(delay) == 27) {
                break;
            }
        } else {
            this_thread::sleep_for(chrono::milliseconds(delay));
        }
    }
}