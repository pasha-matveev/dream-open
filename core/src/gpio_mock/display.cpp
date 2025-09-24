#include "gpio/display.h"

using namespace std;

Display::Display(const char *device, uint8_t address) {}

Display::~Display() {}

void Display::ensure_i2c() {}

void Display::sendCommand(uint8_t cmd) {}

void Display::sendData(const uint8_t *data, size_t size) {}

void Display::init() {}

void Display::clear() {}

void Display::set_pixel(int x, int y, bool color) {}

void Display::flush() {}

void Display::draw_image(const vector<vector<uint8_t>> &img) {}