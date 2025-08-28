#pragma once
#include <vector>

constexpr int WIDTH = 128;
constexpr int HEIGHT = 64;

class Display {
   public:
    Display(const char*, uint8_t);
    ~Display();
    void init();
    void set_pixel(int x, int y, bool color);
    void flush();
    void clear();
    void draw_image(const std::vector<std::vector<uint8_t>>&);

   private:
    void sendCommand(uint8_t cmd);
    void sendData(const uint8_t* data, size_t size);
    void ensure_i2c();

    int i2c_fd;
    std::vector<uint8_t> buffer;
};