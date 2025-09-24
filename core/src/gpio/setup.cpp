#include <wiringPi.h>

void setup_wiringpi() {
    if (wiringPiSetupPinType(WPI_PIN_BCM) == -1) {
        throw runtime_error("Failed to setup wiringPi");
    }
}