/**
 * Main control file for a multi-functional drone project. 
 * This script initializes and continually manages interactions between 
 * various components: Radio Controller, IMU, and ESCs. 
 * The setup function initializes serial communication and 
 * component-specific setup functions. 
 * The main loop handles real-time data processing from the radio 
 * controller, IMU, and executes control algorithms through the ESCs. It schedules and broadcasts telemetry 
 * and debugging information at regular intervals.
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <PID_v1.h>
#include <IMU.h>
#include <RC.h>
#include <ESCs.h>

void setup(void) {
  Serial.begin(115200);   // Initialize serial communication at 115200 bps
  setupRC();              // Setup the Radio Controller
  setupIMU();             // Setup the Inertial Measurement Unit
  setupESCs();            // Setup the Electronic Speed Controllers
  // CalibrateESC();      // Turn on to calibrate the ESCs 
}

void loop(void) {
  channelInterruptHandler();   // Handle incoming signals from the radio controller
  neutralPositionAdjustment(); // Adjust the neutral position based on the radio controller input

  SerialDataWriteESCs();       // Listen for serial commands to update ESC settings

  loopIMU();                   // Process data from the IMU and update orientation estimates

  loopESCs();                  // Execute control loops for the ESCs using the PID algorithm

  // Check if it's time to print serial data for debugging and telemetry
  curr_time = micros();        // Update current time
  if (curr_time - time_prev >= dt_com) {  
    time_prev = micros();      // Reset the timer
    SerialDataPrintRC();       // Print current data from the radio controller
    SerialDataPrintIMU();      // Print current data from the IMU
    SerialDataInput();         // Print current desired inputs
  }
  SerialDataTeleESCs();        // Continuously print telemetry data from the ESCs
}

