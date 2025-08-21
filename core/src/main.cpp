#include <stdio.h>
#include <unistd.h>

#include <chrono>
#include <thread>

#include "camera.h"

using namespace cv;
using namespace std;

int main() {
    Camera camera;
    camera.start();
    while (true) {
        camera.capture();
        camera.preview();
        if (cv::waitKey(1000 / VIDEO_FPS / 2) == 27) {  // 27 = ESC
            break;
        }
    }
}