#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "Proto1_radio_ctrl.h"

// Create an instance of the Proto1_RadioCtrl class
Proto1_RadioCtrl radioCtrl;

void setup() {
    Serial.begin(115200);  // Start the serial communication at 115200 baud rate
    while (!Serial) {
        ; // Wait for the serial port to connect. Needed for native USB port only.
    }
    Serial.println("Radio Controller Test Initialized.");
    Serial.println("Reading channel values...");
}

void loop() {
    radioCtrl.attachInterrupts();
    
    // Apply neutral position adjustments if necessary
    radioCtrl.neutralPosition();

    // Get the current channel values
    const volatile int* channelValues = radioCtrl.getChannelValues();

    // Print the channel values to the Serial Monitor
    radioCtrl.printChannelStatus();

    // Add a short delay before the next read to make the output readable
    delay(100);
}
