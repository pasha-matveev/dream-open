

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include "Const.h"
#include "Robot.h"

constexpr float MIN_VOLTAGE = 12;

Robot robot;
Adafruit_NeoPixel pixels(2, 9, NEO_GRB + NEO_KHZ800);
unsigned long long alive_tm;
unsigned long long test_tm;
bool rgb_led = false;
// bool prog_running = false;
long long cooldown = 0;
bool blue = false;
bool test_state = false;
long long test_state_cooldown = 0;

template <typename T>
T read_data() {
  T result;
  Serial.readBytes(reinterpret_cast<byte*>(&result), sizeof(T));
  return result;
}

template <class T>
void write_data(T var) {
  Serial.write((const byte*)&var, sizeof(var));
}

void setup() {
  // robot.init_kicker = false;
  // robot.init_dribling = false;
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  pixels.show();
  robot.init(pixels);
  robot.stop();
  delay(1000);
  robot.motors.requestVoltageData();
  float voltage = robot.motors.parseMotorVoltage();
  if (voltage < MIN_VOLTAGE) {
    pixels.setPixelColor(1, pixels.Color(255, 0, 0));
    pixels.show();
    delay(1000);
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.show();
  }
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();

  // delay(6000);
  robot.gyro.generate_correction();

  Serial.begin(115200);
  // write_data<char>('X');

  robot.kicker.start();
}

void loop() {
  // robot.dribling.set_speed(100);
  // robot.dribling.run();
  // Serial.println(analogRead(A0));
  // delay(1);

  // if (test_tm < millis())
  // {
  //   robot.direction += M_PI / 2;
  //   test_tm = millis() + 500;
  // }

  // robot.read();

  // robot.rotation = 0;
  // robot.speed = 50;
  // robot.rotation_limit = 30;

  // robot.run();

  robot.read();

  if (Serial.available()) {
    char c = read_data<char>();
    if (c == 'R') {
      write_data<float>(robot.gyro.angle);
      write_data<bool>((millis() - robot.emitter.last_tm) < 500);
      write_data<bool>(robot.kicker.is_charged());
    } else if (c == 'W') {
      robot.direction = read_data<float>();
      robot.speed = read_data<float>();
      robot.rotation = read_data<float>();
      robot.rotation_limit = read_data<float>();
      robot.dribling.set_speed(read_data<int32_t>());
      robot.kicker_force = read_data<int32_t>();
      rgb_led = read_data<bool>();
      robot.pause = read_data<bool>();
    }
    alive_tm = millis() + 1000;
  }

  // if (millis() - robot.emitter.first_tm > 100) {
  //   robot.emitter.reset();
  //   robot.kicker.set_force(robot.kicker_force);
  // }

  bool mega_kick = false;
  if (test_state || millis() <= test_state_cooldown) {
    robot.dribling.set_speed(ROBOT ? 50 : 30);
  } else if (alive_tm < millis()) {
    // usb не работает
    robot.dribling.set_speed(0);
  }
  if (robot.button.state() && millis() > cooldown) {
    if (test_state) {
      mega_kick = true;
      robot.kicker_force = 100;
      test_state = false;
      test_state_cooldown = millis() + 500;
    } else {
      test_state = true;
    }
    cooldown = millis() + 1000;
  }
  robot.button.was_pressed = false;

  // if (robot.kicker.is_charged() && !blue) {
  //   pixels.setPixelColor(1, pixels.Color(0, 0, 255));
  //   blue = true;
  //   pixels.show();
  // } else if (!robot.kicker.is_charged() && blue) {
  //   pixels.setPixelColor(1, pixels.Color(255, 0, 0));
  //   blue = false;
  //   pixels.show();
  // }

  robot.kicker.charge();
  if ((alive_tm > millis() || mega_kick) && robot.init_kicker) {
    if (robot.kicker.is_charged() && robot.kicker_force != 0) {
      robot.kicker.charge(false);
      robot.kicker.kick(robot.kicker_force);
    }
  }
  if (mega_kick) {
    robot.kicker_force = 0;
  }

  if (alive_tm > millis() && !robot.pause) {
    robot.run();
  } else {
    robot.reset();
    robot.stop();
  }

  // if (alive_tm <= millis()) {
  //   rgb_led = false;
  //   robot.button.was_pressed = false;
  //   prog_running = false;
  // }

  // if (rgb_led) {
  //   pixels.setPixelColor(1, pixels.ColorHSV(millis() * 80 % 65535, 255,
  //   255));
  // } else {
  //   pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  // }
  // pixels.show();
}