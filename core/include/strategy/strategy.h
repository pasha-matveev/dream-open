#include "robot.h"
#include "tracking/ball.h"

enum State { INITIAL };

class Strategy {
 private:
  string role;

  void run_keeper(Robot &robot, Ball &ball);
  void run_attacker(Robot &robot, Ball &ball);

 public:
  State state = INITIAL;
  void run(Robot &robot, Ball &ball);
  Strategy();
};