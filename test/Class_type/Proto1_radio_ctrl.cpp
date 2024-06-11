#include "Proto1_radio_ctrl.h"
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

Proto1_RadioCtrl* Proto1_RadioCtrl::instance = nullptr;

Proto1_RadioCtrl::Proto1_RadioCtrl() {
    instance = this; // Set the instance pointer to this object
}

void Proto1_RadioCtrl::neutralPosition() {
  int min = 1590;
  int max = 1610;
  // Roll
  if (receiverValues[0] < max && receiverValues[0] > min) {
    receiverValues[0] = 1600;
  } 
  // Pitch
  if (receiverValues[1] < max && receiverValues[1] > min) {
    receiverValues[1] = 1600;
  } 
  // Yaw
  if (receiverValues[3] < max && receiverValues[3] > min) {
    receiverValues[3] = 1600;
  } 
  if (receiverValues[0] == receiverValues[1] && 
        receiverValues[1] == receiverValues[3] && 
        receiverValues[3] == receiverValues[0]) {
    receiverValues[0] = 1600;
    receiverValues[1] = 1600;
    receiverValues[3] = 1600;
  }
}

// void Proto1_RadioCtrl::neutralPosition() {
//     int min = 1590;
//     int max = 1610;
//     for (int i = 0; i < 4; ++i) {
//         if (receiverValues[i] < max && receiverValues[i] > min) {
//             receiverValues[i] = 1600;
//         }
//     }
// }

void Proto1_RadioCtrl::attachInterrupts() {
    pinMode(channel_pins[0], INPUT_PULLUP);
    pinMode(channel_pins[1], INPUT_PULLUP);
    pinMode(channel_pins[2], INPUT_PULLUP);
    pinMode(channel_pins[3], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(channel_pins[0]), ISRWrapper0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel_pins[1]), ISRWrapper1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel_pins[2]), ISRWrapper2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel_pins[3]), ISRWrapper3, CHANGE);
}

void Proto1_RadioCtrl::ISRWrapper0() { instance->updateChannelValues(0); }
void Proto1_RadioCtrl::ISRWrapper1() { instance->updateChannelValues(1); }
void Proto1_RadioCtrl::ISRWrapper2() { instance->updateChannelValues(2); }
void Proto1_RadioCtrl::ISRWrapper3() { instance->updateChannelValues(3); }

// void Proto1_RadioCtrl::updateChannelValues() {
//     current_time = micros();
//     // Channel 1
//     if (digitalRead(channel_pins[0])) {
//         if (last_channels[0] == 0) {
//             last_channels[0] == 1;
//             timer[0] = current_time;
//         }
//         else {
//             last_channels[0] == 0;
//             receiverValue[0] = current_time - timer[0];
//         }
//     }
//     // Channel 2
//     if (digitalRead(channel_pins[1])) {
//         if (last_channels[1] == 0) {
//             last_channels[1] == 1;
//             timer[1] = current_time;
//         }
//         else {
//             last_channels[1] == 0;
//             receiverValue[1] = current_time - timer[1];
//         }
//     }
//     // Channel 3
//     if (digitalRead(channel_pins[2])) {
//         if (last_channels[2] == 0) {
//             last_channels[2] == 1;
//             timer[2] = current_time;
//         }
//         else {
//             last_channels[2] == 0;
//             receiverValue[2] = current_time - timer[2];
//         }
//     }   
//     // Channel 4
//     if (digitalRead(channel_pins[3])) {
//         if (last_channels[3] == 0) {
//             last_channels[3] == 1;
//             timer[3] = current_time;
//         }
//         else {
//             last_channels[3] == 0;
//             receiverValue[3] = current_time - timer[3];
//         }
//     } 
// }

void Proto1_RadioCtrl::updateChannelValues(int channel) {
    uint32_t current_time = micros();
    bool current_state = digitalRead(channel_pins[channel]);
    if (current_state && !last_channels[channel]) {
        timers[channel] = current_time;
        last_channels[channel] = true;
    } else if (!current_state && last_channels[channel]) {
        receiverValues[channel] = current_time - timers[channel];
        last_channels[channel] = false;
    }
}

const volatile int* Proto1_RadioCtrl::getChannelValues() const {
    return receiverValues;
}

void Proto1_RadioCtrl::printChannelStatus() {
    Serial.print("C1: "); Serial.print(receiverValues[0]);
    Serial.print(", C2: "); Serial.print(receiverValues[1]);
    Serial.print(", C3: "); Serial.print(receiverValues[2]);
    Serial.print(", C4: "); Serial.println(receiverValues[3]);
}
