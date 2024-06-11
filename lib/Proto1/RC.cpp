#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <RC.h>

// Timing variables used for managing intervals and tracking time-stamps of various operations like MPU updates and communications
unsigned long curr_time = 0;      // Stores the current time snapshot, updated frequently within the loop or interrupt routines
unsigned long prev_time_com = 0;  // Last time when a specific communication or operation was executed
unsigned long dt_com = 50000;     // Interval in microseconds for periodic communications or operations
unsigned long prev_time_mpu = 0;  // Last time the MPU data was processed
unsigned long dt_mpu = 0;         // Desired interval between MPU data processing, typically set based on sensor sampling rate
unsigned long time_prev = 0;      // General-purpose time stamp used for various delay and timing checks

// Variables to hold the current and last recorded times for channel interrupts
volatile uint32_t current_time;
uint32_t last_channel_1 = 0;
uint32_t last_channel_2 = 0;
uint32_t last_channel_3 = 0;
uint32_t last_channel_4 = 0;

// Timers for measuring pulse widths from the RC receiver
uint32_t timer_1;
uint32_t timer_2;
uint32_t timer_3;
uint32_t timer_4;

double ReceiverValue[4]; // Increase the array size to 4 for Channel 1 to Channel 4

// Interrupt service routine for reading RC receiver channel inputs
void channelInterruptHandler() {
  current_time = micros();  // Capture the current time at the start of the interrupt

  // Channel 1 processing
  if (digitalRead(channel_1_pin)) {
    if (last_channel_1 == 0) {
      last_channel_1 = 1;
      timer_1 = current_time;  // Start timing the pulse width
    }
  } else if (last_channel_1 == 1) {
    last_channel_1 = 0;
    ReceiverValue[0] = current_time - timer_1;  // Calculate pulse width
  }

  // Channel 2 processing
  if (digitalRead(channel_2_pin)) {
    if (last_channel_2 == 0) {
      last_channel_2 = 1;
      timer_2 = current_time;
    }
  } else if (last_channel_2 == 1) {
    last_channel_2 = 0;
    ReceiverValue[1] = current_time - timer_2;
  }

  // Channel 3 processing
  if (digitalRead(channel_3_pin)) {
    if (last_channel_3 == 0) {
      last_channel_3 = 1;
      timer_3 = current_time;
    }
  } else if (last_channel_3 == 1) {
    last_channel_3 = 0;
    ReceiverValue[2] = current_time - timer_3;
  }

  // Channel 4 processing
  if (digitalRead(channel_4_pin)) {
    if (last_channel_4 == 0) {
      last_channel_4 = 1;
      timer_4 = current_time;
    }
  } else if (last_channel_4 == 1) {
    last_channel_4 = 0;
    ReceiverValue[3] = current_time - timer_4;
  }
}

// Function to adjust receiver values to neutral position if within a certain range
void neutralPositionAdjustment() {
  int min = 1490;
  int max = 1510;

  // Neutral position adjustments
  if (ReceiverValue[0] < max && ReceiverValue[0] > min) {
    ReceiverValue[0] = 1500;
  }
  if (ReceiverValue[1] < max && ReceiverValue[1] > min) {
    ReceiverValue[1] = 1500;
  }
  if (ReceiverValue[3] < max && ReceiverValue[3] > min) {
    ReceiverValue[3] = 1500;
  }

  // Synchronize neutral positions if they are similar
  if (ReceiverValue[0] == ReceiverValue[1] && ReceiverValue[1] == ReceiverValue[3] && ReceiverValue[3] == ReceiverValue[0]) {
    ReceiverValue[0] = 1500;
    ReceiverValue[1] = 1500;
    ReceiverValue[3] = 1500;
  }
}

// Setup function to configure RC receiver input pins and interrupts
void setupRC() {
  pinMode(channel_1_pin, INPUT_PULLUP);
  pinMode(channel_2_pin, INPUT_PULLUP);
  pinMode(channel_3_pin, INPUT_PULLUP);
  pinMode(channel_4_pin, INPUT_PULLUP);

  // Attach interrupts for all RC channels
  attachInterrupt(digitalPinToInterrupt(channel_1_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_2_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_3_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_4_pin), channelInterruptHandler, CHANGE);
}


// Function to print receiver values to the Serial Monitor
void SerialDataPrintRC() {
  Serial.print("C1: ");
  Serial.print(ReceiverValue[0]);
  Serial.print(" - ");
  Serial.print("C2: ");
  Serial.print(ReceiverValue[1]);
  Serial.print(" - ");
  Serial.print("C3: ");
  Serial.print(ReceiverValue[2]);
  Serial.print(" - ");
  Serial.print("C4: ");
  Serial.println(ReceiverValue[3]);
}