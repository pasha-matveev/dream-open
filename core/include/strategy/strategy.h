#include "robot.h"

enum State { INITIAL };

class Strategy {
    public:
    State state = INITIAL;
    void run(Robot &robot);
};