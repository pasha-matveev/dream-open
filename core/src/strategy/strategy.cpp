#include "strategy/strategy.h"

#include <spdlog/spdlog.h>

#include "utils/vec.h"

void Strategy::run(Robot &robot) {
    spdlog::info(robot.camera->ball.angle());
    // robot.rotation = normalize_angle(.rotation + robot.camera->ball.angle());
    // spdlog::info(robot.camera->ball.point.x);
    // spdlog::info(robot.camera->ball.point.y);
    // robot.rotation = robot.camera->ball.visible;
    // robot.speed = 100;
    // cout << "strategy" << endl;
}