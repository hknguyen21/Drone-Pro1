#include "Proto1_IMU.h"
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

Proto1_IMU::Proto1_IMU() : dt_com(50000), r_to_d(180 / 3.141592654), 
    Calib_Roll(0), Calib_Pitch(0), Calib_Yaw(0), 
    AccX_calib(0), AccY_calib(0), AccZ_calib(0) {}

void Proto1_IMU::setup() {
    Serial.begin(115200);
    while (!Serial);
    Wire.begin();
    Wire.setClock(400000);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    for (int i = 0; i < 2000; i++) {
        gyro_signals();
        Calib_Roll += RateRoll;
        Calib_Pitch += RatePitch;
        Calib_Yaw += RateYaw;
    }
    Calib_Roll /= 2000;
    Calib_Pitch /= 2000;
    Calib_Yaw /= 2000;

    prev_time_mpu = micros();
    prev_time_com = prev_time_mpu;
}

void Proto1_IMU::update() {
    curr_time = micros();
    gyro_signals();
    acc_signals();

    RateRoll -= Calib_Roll;
    RatePitch -= Calib_Pitch;
    RateYaw -= Calib_Yaw;

    dt_mpu = (curr_time - prev_time_mpu);
    prev_time_mpu = curr_time;

    cal_angles();

    if (curr_time - prev_time_com >= dt_com) {
        prev_time_com = curr_time;
        printAngles();
    }
}

void Proto1_IMU::printAngles() {
    Serial.print(curr_time / 1000); Serial.print(", ");
    Serial.print("Roll: "); Serial.print(Total_angle[0]); Serial.print(", ");
    Serial.print("Pitch: "); Serial.print(Total_angle[1]); Serial.print(", ");
    Serial.print("Yaw: "); Serial.print(Total_angle[2]); Serial.println("");
}

void Proto1_IMU::acc_signals() {
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(); 
    Wire.requestFrom(0x68,6);
    AccXLSB = Wire.read() << 8 | Wire.read();
    AccYLSB = Wire.read() << 8 | Wire.read();
    AccZLSB = Wire.read() << 8 | Wire.read();
    AccX = (float)AccXLSB / 16384.0;
    AccY = (float)AccYLSB / 16384.0;
    AccZ = (float)AccZLSB / 16384.0;
}

void Proto1_IMU::gyro_signals() {
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);             // Enable Low-pass filter
    Wire.write(0x05);
    Wire.endTransmission(); // End Transmission
    Wire.beginTransmission(0x68);
    Wire.write(0x1B); // Change Gyro sensitivity
    Wire.write(0x8);
    Wire.endTransmission();
    Wire.beginTransmission(0x68); // Access register storing Gyro data
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6); // Read the Gyro data
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    RateRoll = (float)GyroX / 131.0;
    RatePitch = (float)GyroY / 131.0;
    RateYaw = (float)GyroZ / 131.0;
}

void Proto1_IMU::cal_angles() {
    Acc_angle[0] = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * r_to_d;
    Acc_angle[1] = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * r_to_d;

    Gyro_angle[0] += RateRoll * dt_mpu / 1000000;
    Gyro_angle[1] += RatePitch * dt_mpu / 1000000;
    Gyro_angle[2] += RateYaw * dt_mpu / 1000000;

    Total_angle[0] = 0.98 * (Total_angle[0] + RateRoll * dt_mpu / 1000000) + 0.02 * Acc_angle[0];
    Total_angle[1] = 0.98 * (Total_angle[1] + RatePitch * dt_mpu / 1000000) + 0.02 * Acc_angle[1];
    Total_angle[2] = Gyro_angle[2];  // Assuming no correction from accelerometer for yaw
}
