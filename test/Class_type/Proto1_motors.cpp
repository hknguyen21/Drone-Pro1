#include "Proto1_motors.h"
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

Proto1_Motors::Proto1_Motors() {}

void Proto1_Motors::attachMotors() {
    Serial.begin(115200);
    while (!Serial);
    motor1.attach(motor1_pin, 1000, 2000);
    motor2.attach(motor2_pin, 1000, 2000);
    motor3.attach(motor3_pin, 1000, 2000);
    motor4.attach(motor4_pin, 1000, 2000);
    resetMotors();
}

void Proto1_Motors::updateMotors(int throttle, float roll_input, float pitch_input, float yaw_input) {
    if (throttle > 1800) {
        throttle = 1800;
    }
    
    motor_input1 = throttle - roll_input + pitch_input + yaw_input; // Front left - counterclockwise
    motor_input2 = throttle - roll_input - pitch_input - yaw_input; // Front right - clockwise
    motor_input3 = throttle + roll_input - pitch_input + yaw_input; // Rear right - counterclockwise
    motor_input4 = throttle + roll_input + pitch_input - yaw_input; // Rear left  - clockwise

    // Throttle Max Limit (2000)
    if (motor_input1 > 2000) motor_input1 = 1990;
    if (motor_input2 > 2000) motor_input2 = 1990;
    if (motor_input3 > 2000) motor_input3 = 1990;
    if (motor_input4 > 2000) motor_input4 = 1990;

    // Throttle Idle (1150)
    if (motor_input1 < 1150) motor_input1 = 1150;
    if (motor_input2 < 1150) motor_input2 = 1150;
    if (motor_input3 < 1150) motor_input3 = 1150;
    if (motor_input4 < 1150) motor_input4 = 1150;

    setMotorOutputs(motor_input1, motor_input2, motor_input3, motor_input4);
}

void Proto1_Motors::setMotorOutputs(float motor1_in, float motor2_in, float motor3_in, float motor4_in) {
    motor1.writeMicroseconds(motor1_in);
    motor2.writeMicroseconds(motor2_in);
    motor3.writeMicroseconds(motor3_in);
    motor4.writeMicroseconds(motor4_in);
}

void Proto1_Motors::resetMotors() {
    motor1.write(0);
    motor2.write(0);
    motor3.write(0);
    motor4.write(0);
}

void Proto1_Motors::printMotorStatus() {
    if (micros() - time_prev >= 20000) {
        time_prev = micros();
        Serial.print(millis());
        Serial.print(", "); Serial.print(motor_input1);
        Serial.print(", "); Serial.print(motor_input2);
        Serial.print(", "); Serial.print(motor_input3);
        Serial.print(", "); Serial.println(motor_input4);
    }
}
