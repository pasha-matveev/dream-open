#include "utils/uart.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <thread>
#include <iostream>

#include "utils/config.h"

using namespace std;

#ifdef __linux__

optional<string> autodetect_usb_serial_port() {
  unique_ptr<udev, decltype(&udev_unref)> udev_ctx(udev_new(), &udev_unref);
  if (!udev_ctx) {
    spdlog::error(
        "udev initialization failed; cannot auto-detect Arduino port");
    return nullopt;
  }

  unique_ptr<udev_enumerate, decltype(&udev_enumerate_unref)> enumerate(
      udev_enumerate_new(udev_ctx.get()), &udev_enumerate_unref);
  if (!enumerate) {
    spdlog::error(
        "udev enumerate creation failed; cannot auto-detect Arduino port");
    return nullopt;
  }

  if (udev_enumerate_add_match_subsystem(enumerate.get(), "tty") < 0) {
    spdlog::error("udev enumerate failed to match tty subsystem");
    return nullopt;
  }

  if (udev_enumerate_scan_devices(enumerate.get()) < 0) {
    spdlog::error("udev enumerate scan failed");
    return nullopt;
  }

  for (auto entry = udev_enumerate_get_list_entry(enumerate.get()); entry;
       entry = udev_list_entry_get_next(entry)) {
    const char* syspath = udev_list_entry_get_name(entry);
    if (!syspath) {
      continue;
    }

    unique_ptr<udev_device, decltype(&udev_device_unref)> device(
        udev_device_new_from_syspath(udev_ctx.get(), syspath),
        &udev_device_unref);
    if (!device) {
      continue;
    }

    const char* devnode = udev_device_get_devnode(device.get());
    if (devnode == nullptr) {
      continue;
    }

    const char* id_bus = udev_device_get_property_value(device.get(), "ID_BUS");
    if (id_bus == nullptr || string(id_bus) != "usb") {
      continue;
    }

    const char* description_raw =
        udev_device_get_property_value(device.get(), "ID_MODEL_FROM_DATABASE");
    if (description_raw == nullptr) {
      description_raw =
          udev_device_get_property_value(device.get(), "ID_MODEL");
    }
    if (description_raw == nullptr) {
      continue;
    }

    string description(description_raw);
    replace(description.begin(), description.end(), '_', ' ');

    if (description == "CH340 serial converter") {
      spdlog::info("Arduino found on {}", devnode);
      return string(devnode);
    }
  }

  spdlog::error("Cannot find Arduino");
  return nullopt;
}

#else

optional<string> autodetect_usb_serial_port() {
  return "/dev/ttyUSB0";
}

#endif

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
  string device = config.serial.device;

  if (device.empty() || device == "auto") {
    auto autodetected = autodetect_usb_serial_port();
    if (!autodetected.has_value()) {
      throw runtime_error("Failed to auto-detect Arduino serial port");
    }
    device = autodetected.value();
  }

  if (device.empty()) {
    throw runtime_error("Serial device is not specified");
  }

  spdlog::info("Opening serial port {}", device);
  serial.Open(device);
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
