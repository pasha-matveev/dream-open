#include "gpio/display.h"  // Убедитесь, что путь к заголовочному файлу верный

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <stdexcept>

using namespace std;

// Конструктор: исправлен размер буфера
Display::Display(const char *device, uint8_t address)
    : i2c_fd(-1), buffer(WIDTH * HEIGHT / 8, 0) {
    i2c_fd = open(device, O_RDWR);
    if (i2c_fd < 0) {
        throw runtime_error("Failed to open the i2c bus");
    }

    if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
        close(i2c_fd);
        i2c_fd = -1;
        throw runtime_error(
            "Failed to acquire bus access and/or talk to slave");
    }
}

Display::~Display() {
    if (i2c_fd >= 0) {
        // Перед закрытием очистим экран
        try {
            clear();
            flush();
        } catch (const exception &e) {
            // Игнорируем ошибки при очистке в деструкторе
        }
        close(i2c_fd);
    }
}

void Display::ensure_i2c() {
    if (i2c_fd < 0) {
        throw runtime_error("i2c filedriver not connected");
    }
}

void Display::sendCommand(uint8_t cmd) {
    ensure_i2c();
    uint8_t buffer[2] = {0x00, cmd};
    if (write(i2c_fd, buffer, 2) != 2) {
        throw runtime_error("Failed to write command to the i2c bus");
    }
}

void Display::sendData(const uint8_t *data, size_t size) {
    ensure_i2c();
    if (!data || size == 0) {
        throw logic_error("Data is required");
    }

    vector<uint8_t> write_buffer(size + 1);
    write_buffer[0] = 0x40;  // Управляющий байт для данных
    copy(data, data + size, write_buffer.begin() + 1);

    if (write(i2c_fd, write_buffer.data(), write_buffer.size()) !=
        (ssize_t)write_buffer.size()) {
        throw runtime_error("Failed to write data to the i2c bus");
    }
}

void Display::init() {
    // Последовательность команд для инициализации SSD1306
    sendCommand(0xAE);  // Display OFF
    sendCommand(0xD5);  // Set Display Clock Divide Ratio/Oscillator Frequency
    sendCommand(0x80);  //...
    sendCommand(0xA8);  // Set MUX Ratio
    sendCommand(HEIGHT - 1);
    sendCommand(0xD3);        // Set Display Offset
    sendCommand(0x00);        // No offset
    sendCommand(0x40 | 0x0);  // Set Start Line
    sendCommand(0x8D);        // Charge Pump Setting
    sendCommand(0x14);        // Enable charge pump
    sendCommand(0x20);        // Set Memory Addressing Mode
    sendCommand(0x00);        // Horizontal Addressing Mode
    sendCommand(0xA1);        // Set Segment Re-map
    sendCommand(0xC8);        // Set COM Output Scan Direction
    sendCommand(0xDA);        // Set COM Pins Hardware Configuration
    sendCommand(0x12);
    sendCommand(0x81);  // Set Contrast Control
    sendCommand(0xCF);
    sendCommand(0xD9);  // Set Pre-charge Period
    sendCommand(0xF1);
    sendCommand(0xDB);  // Set VCOMH Deselect Level
    sendCommand(0x40);
    sendCommand(0xA4);  // Entire Display ON from RAM
    sendCommand(0xA6);  // Set Normal Display
    sendCommand(0xAF);  // Display ON
}

void Display::clear() { std::fill(buffer.begin(), buffer.end(), 0); }

void Display::set_pixel(int x, int y, bool color) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
        throw logic_error("Point out of bounds");
    }
    int index = x + (y / 8) * WIDTH;
    uint8_t bit_mask = 1 << (y % 8);
    if (color) {
        buffer[index] |= bit_mask;
    } else {
        buffer[index] &= ~bit_mask;
    }
}

void Display::flush() {
    // Устанавливаем диапазон столбцов и страниц
    sendCommand(0x21);              // Set Column Address
    sendCommand(0);                 // Column Start Address
    sendCommand(WIDTH - 1);         // Column End Address
    sendCommand(0x22);              // Set Page Address
    sendCommand(0);                 // Page Start Address
    sendCommand((HEIGHT / 8) - 1);  // Page End Address

    // Отправляем весь наш кадровый буфер на дисплей.
    sendData(buffer.data(), buffer.size());
}

void Display::draw_image(const vector<vector<uint8_t>> &img) {
    clear();
    for (int i = 0; i < HEIGHT; ++i) {
        for (int j = 0; j < WIDTH; ++j) {
            set_pixel(j, i, !img[i][j]);
        }
    }
    flush();
}