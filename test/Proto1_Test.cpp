#include <Arduino.h>
#include <ESP32Servo.h> // Change to the standard Servo library for ESP32
#include <Wire.h>
#include <PID_v1.h>
#include <IMU.h>
#include <RC.h>
#include <ESCs.h>

void setup(void) {
  Serial.begin(115200);
  setup_RC();
  setup_IMU();
  setupESCs();
}

void loop(void) {
  /********* Radio Controller *************/  
  channelInterruptHandler();
  neutralPositionAdjustment();

  /********* IMU *************/
  loop_IMU();
  
  // /********* ESCs (PID) *************/  
  loopESCs();

  
  curr_time = micros();
  if (curr_time - time_prev >= 20000)
    {
      time_prev = micros();
      SerialDataWriteRC();
      SerialDataWriteESCs();
      SerialDataWriteIMU();
    }
}
