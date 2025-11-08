#pragma once

#include <assert.h>
#include <libserial/SerialStream.h>

#include <chrono>
#include <cstring>
#include <thread>

using namespace LibSerial;
using namespace std;

class UART {
 private:
  SerialStream serial;

 public:
  void connect();
  void wait_for_x();

  template <class T>
  T read_data() {
    assert(serial.good());
    while (!serial.IsDataAvailable()) {
      std::this_thread::sleep_for(chrono::milliseconds(10));
    }
    const int SZ = sizeof(T);
    char buffer[SZ];
    serial.read(buffer, SZ);
    assert(serial.good());
    T res;
    memcpy(&res, buffer, SZ);
    return res;
  };

  template <class T>
  void write_data(T val) {
    assert(serial.good());
    const int SZ = sizeof(T);
    char buffer[SZ];
    memcpy(buffer, &val, SZ);
    serial.write(buffer, SZ);
    assert(serial.good());
    serial.flush();
    assert(serial.good());
  };
};