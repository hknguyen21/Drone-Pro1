#ifndef PROTO1_MOTORS_H
#define PROTO1_MOTORS_H

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

class Proto1_Motors {
public:
    Proto1_Motors();
    void attachMotors();
    void updateMotors(int throttle, float roll_input, float pitch_input, float yaw_input);
    void setMotorOutputs(float motor1_in, float motor2_in, float motor3_in, float motor4_in);
    void resetMotors();
    void printMotorStatus();

private:
    Servo motor1, motor2, motor3, motor4;
    float motor_input1, motor_input2, motor_input3, motor_input4;
    // const int motor1_pin = 4;
    // const int motor2_pin = 18;
    // const int motor3_pin = 19;
    // const int motor4_pin = 23;
    const int motor1_pin = 23;
    const int motor2_pin = 4;
    const int motor3_pin = 13;
    const int motor4_pin = 32;
    unsigned long time_prev = 0; // Variable used for serial monitoring
};

#endif
