#include <wiringPi.h>

using namespace std;

void setup_wiringpi() {
    if (wiringPiSetupPinType(WPI_PIN_BCM) == -1) {
        throw runtime_error("Failed to setup wiringPi");
    }
}