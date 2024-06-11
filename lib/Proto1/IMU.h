#ifndef IMU_H
#define IMU_H

// Necessary external variables
extern unsigned long curr_time, prev_time_com, dt_com;
extern unsigned long prev_time_mpu, dt_mpu, time_prev;

// IMU measurement variables
extern double RateRoll, RatePitch, RateYaw;
extern double Total_angle[3]; // Complementary filter results

// Function Declarations
void setupIMU();
void loopIMU();
void accSignals();
void gyroSignals();
void SerialDataPrintIMU();

#endif // IMU_H
