#include "robot.h"
#include "tracking/ball.h"

enum State { INITIAL };

class Strategy {
 public:
  State state = INITIAL;
  void run(Robot &robot, Ball &ball);
};