#include "utils/sleep.h"

#include <chrono>
#include <thread>

using namespace std;

void sleep(int ms) { this_thread::sleep_for(chrono::milliseconds(ms)); }