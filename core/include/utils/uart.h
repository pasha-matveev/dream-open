#pragma once

#include <assert.h>
#include <libserial/SerialStream.h>

#include <chrono>
#include <cstring>
#include <optional>
#include <thread>

#include "utils/init_guard.h"

using namespace LibSerial;
using namespace std;

class UART {
 private:
  SerialStream serial;

 public:
  ~UART();
  void connect();
  void wait_for_x(std::optional<InitDeadline> deadline = std::nullopt);
  void disconnect();

  // Во время init_hardware вызывается с deadline — poll-цикл прерывается по
  // таймауту/сигналу. В главном цикле deadline отсутствует — поведение прежнее.
  template <class T>
  T read_data(std::optional<InitDeadline> deadline = std::nullopt,
              const char* stage = "uart read") {
    assert(serial.good());
    while (!serial.IsDataAvailable()) {
      if (deadline) check_init_deadline(*deadline, stage);
      std::this_thread::sleep_for(chrono::microseconds(100));
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
