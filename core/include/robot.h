#include <libserial/SerialStream.h>

using namespace LibSerial;

class Robot {
   private:
    SerialStream serial;

   public:
    Robot();
    void update_from_arduino() ;
};