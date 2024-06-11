#include <Arduino.h>
#include <Proto1_motors.h>
#include <Wire.h>
#include <ESP32Servo.h>

Proto1_Motors motors;

void setup() {
    motors.attachMotors(); // Initialize all motors
    Serial.println("Enter a throttle value (1000-2000) to set the speed for all motors:");
}

void loop() {
    if (Serial.available() > 0) {
        // Read the incoming string
        String input = Serial.readStringUntil('\n');
        input.trim();  // Remove any extraneous whitespace

        // Convert input to an integer
        int throttle = input.toInt();

        motors.updateMotors(throttle, 0, 0, 0);  // No roll, pitch, or yaw input

        Serial.print("All motors set to throttle: ");
        Serial.println(throttle);
    }
    motors.printMotorStatus();  // Optionally print motor status to verify
}
