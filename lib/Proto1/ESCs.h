#ifndef ESCS_H
#define ESCS_H

// Motor pin constants
const int mot1_pin = 23;
const int mot2_pin = 4;
const int mot3_pin = 13;
const int mot4_pin = 32;

// Function Declarations
void setupESCs();
void resetPID();
void loopESCs();
void SerialDataPrintESCs();
void SerialDataInput();
void SerialDataWriteESCs();
void SerialDataTeleESCs();
void CalibrateESC();
void WaitForKeyStroke();
double convertStringToDouble(String str);

#endif // ESCS_H
