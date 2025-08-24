#pragma once

#include <libserial/SerialStream.h>

#include <tuple>

using namespace LibSerial;
using namespace std;

class UART {
   private:
    SerialStream serial;

   public:
    void connect();
    tuple<float, bool, bool> read_data();
};