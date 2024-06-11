#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <PID_v1.h>
#include <RC.h>
#include <IMU.h>
#include <ESCs.h>

// Initialize servo objects for each motor
Servo mot1, mot2, mot3, mot4;

// Initialize motor control inputs to zero
float MotorInput1 = 0, MotorInput2 = 0, MotorInput3 = 0, MotorInput4 = 0;

// Max desired angle for roll and pitch and rate for yaw
double maxAngle = 10;   // degrees
double maxRate = 10;    // degrees per second

// PID control gains for roll
// double PRateRoll = 2.5;
// double IRateRoll = 0;
// double DRateRoll = 0.03;

double PRateRoll = 2;
double IRateRoll = 0;
double DRateRoll = 0.01;

// PID control gains for pitch (linked to roll's gains initially) and yaw 
double PRatePitch = PRateRoll, PRateYaw = 0;
double IRatePitch = IRateRoll, IRateYaw = 0;
double DRatePitch = DRateRoll, DRateYaw = 0;

// PID control gains for angle control (roll, pitch, yaw)
// double PAngleRoll = 2.27;
// double IAngleRoll = 0.0042;
// double DAngleRoll = 0;

double PAngleRoll = 4;
double IAngleRoll = 0;
double DAngleRoll = 0;

double PAnglePitch = PAngleRoll, PAngleYaw = 0;
double IAnglePitch = IAngleRoll, IAngleYaw = 0;
double DAnglePitch = DAngleRoll, DAngleYaw = 0;

// Define the signal boundaries and center
const int signalMin = 1250;
const int signalMax = 1750;
const int signalCenter = 1500;

// Desired orientation angles based on receiver input, used for angle PID control
double DesiredAngleRoll = 0, DesiredAnglePitch = 0, DesiredAngleYaw = 0;

// Desired rotation rates derived from the desired angles, used for rate PID control
double DesiredRateRoll = 0, DesiredRatePitch = 0, DesiredRateYaw = 0;

// Control inputs for roll, pitch, and yaw based on PID calculations; these values are applied to ESCs
double InputRoll = 0, InputThrottle = 0, InputPitch = 0, InputYaw = 0;

// The PID object is configured as follows:
// input = sensor, variable to be controller;
// output = pid output, command sent to the motors;
// setpoint = reference setpoint, the desired angle (usually 0 degree to maintain an upward position)
// PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Define PID Controllers for angle control for roll, pitch, and yaw
PID AngleRollPID(&Total_angle[0], &DesiredRateRoll, &DesiredAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, DIRECT);
PID AnglePitchPID(&Total_angle[1], &DesiredRatePitch, &DesiredAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, DIRECT);
//PID AngleYawPID(&Total_angle[2], &DesiredRateYaw, &DesiredAngleYaw, PAngleYaw, IAngleYaw, DAngleYaw, DIRECT);

// Define PID Controllers for rate control for roll, pitch, and yaw
PID RateRollPID(&RateRoll, &InputRoll, &DesiredRateRoll, PRateRoll, IRateRoll, DRateRoll, DIRECT);
PID RatePitchPID(&RatePitch, &InputPitch, &DesiredRatePitch, PRatePitch, IRatePitch, DRatePitch, DIRECT);
PID RateYawPID(&RateYaw, &InputYaw, &DesiredRateYaw, PRateYaw, IRateYaw, DRateYaw, DIRECT);

// Setup function to initialize ESCs and configure PIDs
void setupESCs(void) {
  mot1.attach(mot1_pin, 1000, 2000);
  mot2.attach(mot2_pin, 1000, 2000);
  mot3.attach(mot3_pin, 1000, 2000);
  mot4.attach(mot4_pin, 1000, 2000);

  // Stop ESCs from beeping by initializing them to 0 microseconds
  mot1.write(0);
  mot2.write(0);
  mot3.write(0);
  mot4.write(0);

  // Set PID controllers to automatic mode and configure settings
  AngleRollPID.SetMode(AUTOMATIC);
  AnglePitchPID.SetMode(AUTOMATIC);
  //AngleYawPID.SetMode(AUTOMATIC);
  RateRollPID.SetMode(AUTOMATIC);
  RatePitchPID.SetMode(AUTOMATIC);
  RateYawPID.SetMode(AUTOMATIC);

  AngleRollPID.SetOutputLimits(-200, 200);
  AnglePitchPID.SetOutputLimits(-200, 200);
  //AngleYawPID.SetOutputLimits(-200, 200);
  RateRollPID.SetOutputLimits(-200, 200);
  RatePitchPID.SetOutputLimits(-200, 200);
  RateYawPID.SetOutputLimits(-200, 200);

  AngleRollPID.SetSampleTime(4);
  AnglePitchPID.SetSampleTime(4);
  //AngleYawPID.SetSampleTime(4);
  RateRollPID.SetSampleTime(4);
  RatePitchPID.SetSampleTime(4);
  RateYawPID.SetSampleTime(4);
}

// Main control loop for ESCs
void loopESCs(void) {
  // Calculate desired angles and rates from receiver values (10 degrees)
  DesiredAngleRoll = (ReceiverValue[0] - signalCenter) / (signalMax - signalCenter) * maxAngle; // 1250 - 1750
  DesiredAnglePitch = (ReceiverValue[1] - signalCenter) / (signalMax - signalCenter) * maxAngle; // 1250 - 1750
  InputThrottle = ReceiverValue[2];
  DesiredRateYaw = (ReceiverValue[3] - signalCenter) / (signalMax - signalCenter) * maxRate; // 1250 - 1750

  // Compute PID updates
  AngleRollPID.Compute();
  AnglePitchPID.Compute();
  //AngleYawPID.Compute();

  RateRollPID.Compute();
  RatePitchPID.Compute();
  RateYawPID.Compute();

  // Enforce throttle limits
  if (InputThrottle > 1800) InputThrottle = 1800;
  
  // Calculate motor inputs based on control signals and throttle
  MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw); // Front right - clockwise
  MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw); // Rear right - counterclockwise
  MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw); // Rear left - clockwise
  MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw); // Front left  - counterclockwise

  // Enforce upper motor input limits
  if (MotorInput1 > 2000) { MotorInput1 = 1950; }
  if (MotorInput2 > 2000) { MotorInput2 = 1950; }
  if (MotorInput3 > 2000) { MotorInput3 = 1950; }
  if (MotorInput4 > 2000) { MotorInput4 = 1950; }

  // Enforce lower motor input limits
  int ThrottleIdle = 1050;
  if (MotorInput1 < ThrottleIdle) { MotorInput1 = ThrottleIdle; }
  if (MotorInput2 < ThrottleIdle) { MotorInput2 = ThrottleIdle; }
  if (MotorInput3 < ThrottleIdle) { MotorInput3 = ThrottleIdle; }
  if (MotorInput4 < ThrottleIdle) { MotorInput4 = ThrottleIdle; }

  // Apply cutoff throttle and reset PIDs if necessary
  int ThrottleCutOff = 1000;
  if (ReceiverValue[2] < 1070)
  {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    resetPID();
  }

  // Update ESCs with the new motor inputs
  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);
}

// Prints the current throttle inputs to each of the four motors to the Serial Monitor.
// This function is typically used for debugging purposes to monitor the behavior of motors during operation.
void SerialDataPrintESCs() {
    Serial.print("M1: ");
    Serial.print(MotorInput1);
    Serial.print(" - ");
    Serial.print("M2: ");
    Serial.print(MotorInput2);
    Serial.print(" - ");
    Serial.print("M3: ");
    Serial.print(MotorInput3);
    Serial.print(" - ");
    Serial.print("M4: ");
    Serial.println(MotorInput4);
}

// Prints the desired roll, pitch, and yaw angles and rates to the Serial Monitor.
void SerialDataInput() {
    Serial.print("DRoll: ");
    Serial.print(DesiredAngleRoll);
    Serial.print(" - ");
    Serial.print("DPitch: ");
    Serial.print(DesiredAnglePitch);
    Serial.print(" - ");
    Serial.print("DYaw: ");
    Serial.println(DesiredRateYaw);
    Serial.print("MaxAngle: ");
    Serial.print(maxAngle);
    Serial.print(" - ");
    Serial.print("MaxRate: ");
    Serial.println(maxRate);
    Serial.print("GyroX: ");
    Serial.print(RateRoll);
    Serial.print(" - ");
    Serial.print("GyroY: ");
    Serial.println(RatePitch);
}

// Sends detailed telemetry information over Serial, including PID gain settings and motor inputs.
// This is particularly useful for tuning the PID parameters in real-time, as it provides immediate feedback on the effect of parameter adjustments.
void SerialDataTeleESCs() {
    Serial.print(">P_Angle:");  // Proportional gain for angle PID controllers
    Serial.println(PAngleRoll);
    Serial.print(">I_Angle:");  // Integral gain for angle PID controllers
    Serial.println(IAngleRoll);
    Serial.print(">D_Angle:");  // Derivative gain for angle PID controllers
    Serial.println(DAngleRoll);

    Serial.print(">P_Rate:");   // Proportional gain for rate PID controllers
    Serial.println(PRateRoll);
    Serial.print(">I_Rate:");   // Integral gain for rate PID controllers
    Serial.println(IRateRoll);
    Serial.print(">D_Rate:");   // Derivative gain for rate PID controllers
    Serial.println(DRateRoll);

    Serial.print(">P_Rate_Yaw:");   // Derivative gain for rate PID controllers
    Serial.println(PRateYaw);
    Serial.print(">I_Rate_Yaw:");   // Derivative gain for rate PID controllers
    Serial.println(IRateYaw);

    Serial.print(">Mot1:");     // Current input to motor 1
    Serial.println(MotorInput1);
    Serial.print(">Mot2:");     // Current input to motor 2
    Serial.println(MotorInput2);
    Serial.print(">Mot3:");     // Current input to motor 3
    Serial.println(MotorInput3);
    Serial.print(">Mot4:");     // Current input to motor 4
    Serial.println(MotorInput4);

    Serial.print(">GyroX:");     // Current input to motor 4
    Serial.println(RateRoll);
    Serial.print(">GyroY:");     // Current input to motor 4
    Serial.println(RatePitch);

    Serial.print(">InputRoll:");     // Current input to motor 4
    Serial.println(InputRoll);
}

// This function listens for serial input to dynamically adjust PID parameters based on received commands.
// It is designed to handle different PID settings by reading specific prefixes sent over the serial connection.
void SerialDataWriteESCs() {
    static String received_chars;  // Buffer to store incoming serial data

    // Read all available characters from the serial buffer
    while (Serial.available()) {
        char inChar = (char)Serial.read();  // Read the next character
        received_chars += inChar;          // Append it to the buffer

        // Check if the end of a command line has been reached
        if (inChar == '\n') {
            // Adjust Angle Roll PID parameters based on the prefix
            if (received_chars.startsWith("pa")) {  // Proportional gain for angle roll
                received_chars.remove(0, 2);        // Remove "pa" prefix
                PAngleRoll = convertStringToDouble(received_chars);  // Convert the remaining string to double and assign to PAngleRoll
            }
            else if (received_chars.startsWith("ia")) {  // Integral gain for angle roll
                received_chars.remove(0, 2);        // Remove "ia" prefix
                IAngleRoll = convertStringToDouble(received_chars);  // Convert to double and assign to IAngleRoll
            }
            else if (received_chars.startsWith("da")) {  // Derivative gain for angle roll
                received_chars.remove(0, 2);        // Remove "da" prefix
                DAngleRoll = convertStringToDouble(received_chars);  // Convert to double and assign to DAngleRoll
            }
            // Adjust Rate Roll PID parameters based on the prefix
            else if (received_chars.startsWith("pr")) {  // Proportional gain for rate roll
                received_chars.remove(0, 2);        // Remove "pr" prefix
                PRateRoll = convertStringToDouble(received_chars);  // Convert to double and assign to PRateRoll
            }
            else if (received_chars.startsWith("ir")) {  // Integral gain for rate roll
                received_chars.remove(0, 2);        // Remove "ir" prefix
                IRateRoll = convertStringToDouble(received_chars);  // Convert to double and assign to IRateRoll
            }
            else if (received_chars.startsWith("dr")) {  // Derivative gain for rate roll
                received_chars.remove(0, 2);        // Remove "dr" prefix
                DRateRoll = convertStringToDouble(received_chars);  // Convert to double and assign to DRateRoll
            }
            else if (received_chars.startsWith("p")) {  // Proportional gain for rate yaw
                received_chars.remove(0, 1);        // Remove "p" prefix
                PRateYaw = convertStringToDouble(received_chars);  // Convert to double and assign to DRateRoll
            }
            else if (received_chars.startsWith("i")) {  // Integral gain for rate yaw
                received_chars.remove(0, 1);        // Remove "i" prefix
                IRateYaw = convertStringToDouble(received_chars);  // Convert to double and assign to DRateRoll
            }
            else if (received_chars.startsWith("a")) {  // Max Angle for roll and pitch
                received_chars.remove(0, 1);        // Remove "n" prefix
                maxAngle = convertStringToDouble(received_chars);  // Convert to double and assign to DRateRoll
            }
            else if (received_chars.startsWith("r")) {  // Max Rate for yaw
                received_chars.remove(0, 1);        // Remove "m" prefix
                maxRate = convertStringToDouble(received_chars);  // Convert to double and assign to DRateRoll
            }
            // Clear the buffer for the next command
            received_chars = "";
        }

        PRatePitch = PRateRoll;
        IRatePitch = IRateRoll;
        DRatePitch = DRateRoll;

        PAnglePitch = PAngleRoll;
        IAnglePitch = IAngleRoll;   
        DAnglePitch = DAngleRoll;

        AngleRollPID.SetTunings(PAngleRoll, IAngleRoll, DAngleRoll);
        AnglePitchPID.SetTunings(PAnglePitch, IAnglePitch, DAnglePitch);
        //AngleYawPID.SetTunings(PAngleYaw, IAngleYaw, DAngleYaw);

        RateRollPID.SetTunings(PRateRoll, IRateRoll, DRateRoll);
        RatePitchPID.SetTunings(PRatePitch, IRatePitch, DRatePitch);
        RateYawPID.SetTunings(PRateYaw, IRateYaw, DRateYaw);
    }
}

// Resets all PID controllers by zeroing out desired rates, desired angles, and control inputs.
// This function should be invoked during specific flight conditions, such as after a reset or when a new flight mode is engaged,
// to ensure that the PID controllers start from a known state without residual errors.
void resetPID(void) {
    DesiredRateRoll = 0; DesiredRatePitch = 0; DesiredRateYaw = 0;
    DesiredAngleRoll = 0; DesiredAnglePitch = 0; DesiredAngleYaw = 0;
    InputRoll = 0; InputPitch = 0; InputYaw = 0;
    
    AngleRollPID.SetOutputLimits(-0.01, 0);
    AnglePitchPID.SetOutputLimits(-0.01, 0);
    //AngleYawPID.SetOutputLimits(-0.01, 0);
    RateRollPID.SetOutputLimits(-0.01, 0);
    RatePitchPID.SetOutputLimits(-0.01, 0);
    RateYawPID.SetOutputLimits(-0.01, 0);

    // Compute PID updates
    AngleRollPID.Compute();
    AnglePitchPID.Compute();
    //AngleYawPID.Compute();

    RateRollPID.Compute();
    RatePitchPID.Compute();
    RateYawPID.Compute();

    AngleRollPID.SetOutputLimits(-600, 600);
    AnglePitchPID.SetOutputLimits(-600, 600);
    //AngleYawPID.SetOutputLimits(-600, 600);
    RateRollPID.SetOutputLimits(-600, 600);
    RatePitchPID.SetOutputLimits(-600, 600);
    RateYawPID.SetOutputLimits(-600, 600);
}

void CalibrateESC() {
    // Serial.println("Calibration step 1. Disconnect the battery.");
    // Serial.println("Press any key to continue.");
    // WaitForKeyStroke();
    if (ReceiverValue[2] > 1700) {
        mot1.writeMicroseconds(2000); // Sending MAX_SIGNAL tells the ESC to enter calibration mode
        mot2.writeMicroseconds(2000);
        mot3.writeMicroseconds(2000);
        mot4.writeMicroseconds(2000);
    }
    
    // Serial.println();
    // Serial.println("Calibration step 2. Connect the battery.");
    // Serial.println("Wait for two short bips.");
    // Serial.println("Press any key to continue.");
    // WaitForKeyStroke();

    mot1.writeMicroseconds(1000); // Sending MIN_SIGNAL tells the ESC the calibration value
    mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000);
    mot4.writeMicroseconds(1000);
    Serial.println();
    Serial.println("Wait for 4 short bips, and one long bip.");
    Serial.println("Press any key to finish.");
    WaitForKeyStroke();
}

void WaitForKeyStroke() {
    while (!Serial.available());
    while (Serial.available())
        Serial.read();
}

// Function to convert String to double without losing precision
double convertStringToDouble(String str) {
    char charArray[str.length() + 1];
    str.toCharArray(charArray, str.length() + 1);
    return atof(charArray);
}