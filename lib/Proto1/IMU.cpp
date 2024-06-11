#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <IMU.h>

// Conversion factor from radians to degrees
float r_to_d = 180 / 3.141592654;

// Raw gyroscope and accelerometer data
int16_t GyroX, GyroY, GyroZ;
int16_t AccXLSB, AccYLSB, AccZLSB;

// Processed rate of rotation around each axis
double RateRoll, RatePitch, RateYaw;

// Processed acceleration data
float AccX, AccY, AccZ;

// Arrays to hold the calculated angles from gyro and accelerometer
double Gyro_angle[3] = {0, 0, 0};
double Acc_angle[3] = {0, 0, 0};
double Total_angle[3] = {0, 0, 0};

// Calibration offsets for gyroscope
float Calib_Roll, Calib_Pitch, Calib_Yaw;

// Variable to keep track of the number of samples used in calibration
int RateCalibration;

// Setup the IMU by initializing communication and configuring the device
void setupIMU(void) {
  Wire.begin();  // Start I2C communication
  Wire.setClock(400000);  // Set I2C clock speed to 400 kHz

  // Configure the MPU-6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);  // Wake up MPU-6050
  Wire.endTransmission();

  // Calibrate the gyroscope
  for (RateCalibration = 0; RateCalibration < 2000; RateCalibration++)
  {
    gyroSignals();
    Calib_Roll += RateRoll;
    Calib_Pitch += RatePitch;
    Calib_Yaw += RateYaw;
  }

  // Average the calibration values
  Calib_Roll /= 2000;
  Calib_Pitch /= 2000;
  Calib_Yaw /= 2000;

  // Initialize timing variables for motion processing
  prev_time_mpu = micros();
  prev_time_com = prev_time_mpu;
}

// Main loop to process IMU data
void loopIMU(void) {
  gyroSignals();
  accSignals();

  // Adjust the gyroscope readings by the calibration offsets
  RateRoll -= Calib_Roll;
  RatePitch -= Calib_Pitch;
  RateYaw -= Calib_Yaw;

  // Calculate the time difference since last update
  dt_mpu = curr_time - prev_time_mpu;
  prev_time_mpu = curr_time;

  // Compute angle estimates from accelerometer
  Acc_angle[0] = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * r_to_d - 1.2;
  Acc_angle[1] = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * r_to_d + 1.5;

  // Update gyro angle estimates based on the rate of turn
  Gyro_angle[0] += RateRoll * dt_mpu / 1000000;
  Gyro_angle[1] += RatePitch * dt_mpu / 1000000;
  Gyro_angle[2] += RateYaw * dt_mpu / 1000000;

  // Combine gyro and accelerometer angles using a complementary filter
  Total_angle[0] = 0.98 * (Total_angle[0] + RateRoll * dt_mpu / 1000000) + 0.02 * Acc_angle[0];
  Total_angle[1] = 0.98 * (Total_angle[1] + RatePitch * dt_mpu / 1000000) + 0.02 * Acc_angle[1];
  Total_angle[2] = Gyro_angle[2];  // Yaw angle is typically not corrected by accelerometer
}

// Function to print current IMU angles to the Serial monitor
void SerialDataPrintIMU(void) {
  Serial.print("Roll: ");
  Serial.print(Total_angle[0]);
  Serial.print(" - Pitch: ");
  Serial.print(Total_angle[1]);
  Serial.print(" - Yaw: ");
  Serial.println(Total_angle[2]);
  Serial.println(" ");
}

// Function to read accelerometer data
void accSignals(void) {
  Wire.beginTransmission(0x68);  // Start the communication with the MPU6050
  Wire.write(0x1A);              // Enable Low-pass filter
  Wire.write(0x05);              // Enable Low-pass filter - 10Hz-ish
  Wire.endTransmission();        // End Transmission
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // Access register storing Acc data
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);  // Read the Acc data
  AccXLSB = Wire.read() << 8 | Wire.read();
  AccYLSB = Wire.read() << 8 | Wire.read();
  AccZLSB = Wire.read() << 8 | Wire.read();
  AccX = (float)AccXLSB / 16384.0;
  AccY = (float)AccYLSB / 16384.0;
  AccZ = (float)AccZLSB / 16384.0;
}

// Function to read gyroscope data
void gyroSignals(void) {
  Wire.beginTransmission(0x68);  // Start the communication with the MPU6050
  Wire.write(0x1A);              // Enable Low-pass filter
  Wire.write(0x05);
  Wire.endTransmission();  // End Transmission
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);  // Change Gyro sensitivity
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);  // Access register storing Gyro data
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);  // Read the Gyro data
  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 131.0;
  RatePitch = (float)GyroY / 131.0;
  RateYaw = (float)GyroZ / 131.0;
}