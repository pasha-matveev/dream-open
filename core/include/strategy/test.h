#pragma once

#include <string>

class Robot;
class Object;
class BallTracker;

struct TestContext {
  Robot& robot;
  // Сырое наблюдение мяча с камеры (в т.ч. пиксельная дистанция). Не путать
  // с tracker — это разные сущности: ball — наблюдение текущего кадра,
  // tracker — отфильтрованное состояние с историей.
  Object& ball;
  Object& goal;
  BallTracker& tracker;
};

class TestController {
 public:
  // Возвращает true, если роль найдена и выполнена; false — иначе.
  // Strategy сам решает, что делать с unknown role (логирует ошибку).
  bool run(const std::string& role, TestContext& ctx);
};
