#include <stdio.h>

#include "camera.h"
#include "tracking/ball.h"

using namespace std;

int main() {
    Ball ball;
    Camera camera(ball);
    camera.start();
    while (true) {
        camera.capture();
        camera.preview();
        if (cv::waitKey(1000 / VIDEO_FPS / 2) == 27) {
            break;
        }
    }
}