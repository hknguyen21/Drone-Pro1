#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <RC.h>
#include <IMU.h>

// ================================================================
// Variable declaration
// ================================================================
#define MAX_SIGNAL 2000 // Parameter required for the ESC definition
#define MIN_SIGNAL 1000 // Parameter required for the ESC definition
//#define MOTOR_PIN 13    // Pin 13 attached to the ESC signal pin
//#define POT_PIN 14       // Pin 4 attached to the potentiometer

const int motor1_pin = 23;
const int motor2_pin = 4;
const int motor3_pin = 13;
const int motor4_pin = 32;

Servo ESC1, ESC2, ESC3, ESC4;                   // Define the ESC
double MotorInput;                 // Control Signal. Varies between [0 - 180]
// ================================================================
// Function declaration
// ================================================================
void SerialDataPrint();  // Function to print data on the serial monitor
void Init_Serial();      // Function to init the serial monitor
void WaitForKeyStroke(); // Function to interact with the serial monitor

// ================================================================
// Setup
// ================================================================
void setup()
{
  Init_Serial();                                 // Initialize the serial communication
  setupRC(); 
  ESC1.attach(motor1_pin, MIN_SIGNAL, MAX_SIGNAL); // Initialize the ESC
  ESC2.attach(motor2_pin, MIN_SIGNAL, MAX_SIGNAL); // Initialize the ESC
  ESC3.attach(motor3_pin, MIN_SIGNAL, MAX_SIGNAL); // Initialize the ESC
  ESC4.attach(motor4_pin, MIN_SIGNAL, MAX_SIGNAL); // Initialize the ESC
  Serial.println();
  Serial.println("Calibration step 1. Disconnect the battery.");
  Serial.println("Press any key to continue.");
  WaitForKeyStroke();
  ESC1.writeMicroseconds(MAX_SIGNAL); // Sending MAX_SIGNAL tells the ESC to enter calibration mode
  ESC2.writeMicroseconds(MAX_SIGNAL);
  ESC3.writeMicroseconds(MAX_SIGNAL);
  ESC4.writeMicroseconds(MAX_SIGNAL);

  Serial.println();
  Serial.println("Calibration step 2. Connect the battery.");
  Serial.println("Wait for two short bips.");
  Serial.println("Press any key to continue.");
  WaitForKeyStroke();

  ESC1.writeMicroseconds(MIN_SIGNAL); // Sending MIN_SIGNAL tells the ESC the calibration value
  ESC2.writeMicroseconds(MIN_SIGNAL);
  ESC3.writeMicroseconds(MIN_SIGNAL);
  ESC4.writeMicroseconds(MIN_SIGNAL);
  Serial.println();
  Serial.println("Wait for 4 short bips, and one long bip.");
  Serial.println("Press any key to finish.");
  WaitForKeyStroke();

}

// ================================================================
// Loop
// ================================================================
void loop()
{
//   CtrlPWM = map(analogRead(POT_PIN), 0, 4095, 0, 180); // Read the pot, map the reading from [0, 4095] to [0, 180]
//   ESC1.write(CtrlPWM);                                  // Send the command to the ESC
  channelInterruptHandler();   // Handle incoming signals from the radio controller
  neutralPositionAdjustment(); // Adjust the neutral position based on the radio controller input
  MotorInput = ReceiverValue[2];
  ESC1.writeMicroseconds(MotorInput); 
  ESC2.writeMicroseconds(MotorInput);
  ESC3.writeMicroseconds(MotorInput);
  ESC4.writeMicroseconds(MotorInput);
  SerialDataPrint();                                   // Print data on the serial monitor for debugging
}
// ================================================================
// Function Definition
// ================================================================
void Init_Serial()
{
  Serial.begin(115200);
}
// ================================================================
void SerialDataPrint()
{
  if (micros() - time_prev >= 50000)
  {
    time_prev = micros();
    Serial.print(millis());
    Serial.print("\t");
    SerialDataPrintRC();
    Serial.print("M1: ");
    Serial.print(MotorInput);
    Serial.print(" - ");
    Serial.print("M2: ");
    Serial.print(MotorInput);
    Serial.print(" - ");
    Serial.print("M3: ");
    Serial.print(MotorInput);
    Serial.print(" - ");
    Serial.print("M4: ");
    Serial.println(MotorInput);
  }
}
// ================================================================
void WaitForKeyStroke()
{
  while (!Serial.available())
    ;
  while (Serial.available())
    Serial.read();
}