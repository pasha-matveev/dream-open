#include "utils/millis.h"

#include <chrono>

using namespace std;
using namespace chrono;

long long millis() {
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch())
      .count();
}