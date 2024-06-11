#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <Proto1_IMU.h>

Proto1_IMU imu;

void setup() {
    imu.setup();
}

void loop() {
    imu.update();
}
