#pragma once

#include <chrono>
#include <csignal>
#include <stdexcept>
#include <string>

// Флаг завершения, который ставят обработчики SIGINT/SIGTERM. Определён в
// main.cpp. Объявлен здесь, чтобы блокирующие циклы инициализации железа могли
// прерваться по Ctrl+C, не дожидаясь таймаута.
extern volatile std::sig_atomic_t stop_requested;

using InitDeadline = std::chrono::steady_clock::time_point;

// Вызывается из poll-циклов init_hardware. Кидает исключение, если пришёл
// сигнал завершения или истёк общий бюджет инициализации железа. Раскрутка
// стека доводит исключение до try/catch в main, где деструкторы освобождают
// уже занятые ресурсы (камеру, lidar, serial).
inline void check_init_deadline(InitDeadline deadline, const char* stage) {
  if (stop_requested) {
    throw std::runtime_error(std::string("Hardware init aborted by signal at: ") +
                             stage);
  }
  if (std::chrono::steady_clock::now() > deadline) {
    throw std::runtime_error(std::string("Hardware init timed out at: ") + stage);
  }
}
