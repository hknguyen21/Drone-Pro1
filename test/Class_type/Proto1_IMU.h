#ifndef PROTO1_IMU_H
#define PROTO1_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

class Proto1_IMU {
public:
    Proto1_IMU();  // Constructor
    void setup();  // Setup MPU6050 and calibrate
    void update(); // Update sensor readings and calculate angles
    void printAngles(); // Print angles to Serial

private:
    void acc_signals();
    void gyro_signals();
    void cal_angles();

    int16_t GyroX, GyroY, GyroZ;
    int16_t AccXLSB, AccYLSB, AccZLSB;
    float RateRoll, RatePitch, RateYaw;
    float AccX, AccY, AccZ;
    unsigned long prev_time_mpu, dt_mpu;
    unsigned long curr_time, prev_time_com, dt_com;
    float Gyro_angle[3];
    float Acc_angle[3];
    float Total_angle[3];
    volatile float Calib_Roll, Calib_Pitch, Calib_Yaw;
    volatile float AccX_calib, AccY_calib, AccZ_calib;
    float r_to_d;
};

#endif
