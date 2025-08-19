#include "Gyro.h"

void Gyro::init() {
    Wire.begin();
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);

    mpu.CalibrateGyro(6);

    // mpu.setXAccelOffset(1518);
    // mpu.setYAccelOffset(-3762);
    // mpu.setZAccelOffset(-2238);
    // mpu.setXGyroOffset(24);
    // mpu.setYGyroOffset(-2);
    // mpu.setZGyroOffset(-11);
}

void Gyro::generate_correction() {
    read();
    correction = angle;
}

void Gyro::read() {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        angle = ypr[0];
        angle += M_PI;
        angle *= -1;

        angle -= correction;

        angle = trim(angle);
    }
}

float Gyro::trim(float raw_angle) {
    float mod = 2 * M_PI;
    if (raw_angle >= 0) {
        int k = raw_angle / mod;
        raw_angle -= k * mod;
    } else {
        int k = (-raw_angle + mod - 1) / mod;
        raw_angle += k * mod;
    }
    return raw_angle;
}

float Gyro::relative(float abs_angle) {
    abs_angle = trim(abs_angle);
    float res = abs_angle - angle;
    return trim(res);
}

void Gyro::print_offsets() {
    Serial.println("offsets:");
    mpu.PrintActiveOffsets();
}