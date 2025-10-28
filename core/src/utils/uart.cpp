#include "utils/uart.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <iostream>
#include <thread>

#include "utils/config.h"

using namespace std;

BaudRate int_to_baud(int baud) {
  switch (baud) {
    case 115200:
      return BaudRate::BAUD_115200;
    case 9600:
      return BaudRate::BAUD_9600;
    default:
      throw runtime_error("Unsupported baud rate");
  }
}

void UART::connect() {
  serial.Open(config.serial.device);
  serial.SetBaudRate(int_to_baud(config.serial.rate));
  serial.SetDTR(true);
  serial.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
  serial.SetParity(Parity::PARITY_NONE);
  serial.SetStopBits(StopBits::STOP_BITS_1);

  if (!serial.IsOpen() || !serial.good()) {
    throw runtime_error("Failed to open serial");
  }
}

void UART::wait_for_x() {
  if (!config.serial.interference) {
    spdlog::info("Skip init byte");
    return;
  }
  spdlog::info("Waiting for init byte...");
  while (true) {
    while (!serial.IsDataAvailable()) {
      std::this_thread::sleep_for(chrono::milliseconds(20));
    }
    char buffer[1];
    serial.read(buffer, 1);
    if (buffer[0] == 'X') {
      spdlog::info("Received begin byte (X)");
      break;
    } else {
      spdlog::info("Received random byte ...");
    }
  }
}
