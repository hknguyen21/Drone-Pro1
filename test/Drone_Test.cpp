#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h> // Change to the standard Servo library for ESP32
#include <PID_v1.h>

/********* ESCs *************/
//Servo Motors
Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;
// Motor pins
const int mot1_pin = 23;
const int mot2_pin = 4;
const int mot3_pin = 13;
const int mot4_pin = 32;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

/********* Radio Controller *************/
volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;

volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;

double ReceiverValue[4]; // Increase the array size to 6 for Channel 1 to Channel 6
const int channel_1_pin = 19;
const int channel_2_pin = 18;
const int channel_3_pin = 17;
const int channel_4_pin = 16;

/********* IMU *************/
//IMU declare
float r_to_d = 180 / 3.141592654;;
int16_t GyroX, GyroY, GyroZ;
int16_t AccXLSB, AccYLSB, AccZLSB;
double RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float Calib_Roll, Calib_Pitch, Calib_Yaw;
// volatile float Calib_AccX, Calib_AccY, Calib_AccZ;
double Gyro_angle[3];
double Acc_angle[3];
double Total_angle[3];

int RateCalibrationNumber;

/********* ESCs (PID) *************/
double DesiredAngleRoll, DesiredAnglePitch,DesiredAngleYaw;
double DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
double ErrorAngleRoll, ErrorAnglePitch, ErrorAngleYaw;
double ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
double InputRoll, InputThrottle, InputPitch, InputYaw;

// Gain of Rate Roll
 double PRateRoll = 0.75; //For outdoor flights, keep this gain to 0.75 and for indoor flights keep the gain to be 0.6
 double IRateRoll = 0.012;
 double DRateRoll = 0.0085;
// Gain of Rate Pitch
 double PRatePitch = PRateRoll;
 double IRatePitch = IRateRoll;
 double DRatePitch = DRateRoll;
// Gain of Rate Yaw
 double PRateYaw = 4.2;
 double IRateYaw = 2.8;
 double DRateYaw = 0;

// Gain of Angle
double PAngleRoll = 2;  
double IAngleRoll = 0;  
double DAngleRoll = 0;  

double PAnglePitch = PAngleRoll;
double IAnglePitch = IAngleRoll;
double DAnglePitch = DAngleRoll;

double PAngleYaw = 0;
double IAngleYaw = 0;
double DAngleYaw = 0;

PID AngleRollPID(&Total_angle[0], &DesiredRateRoll, &DesiredAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, DIRECT);
PID AnglePitchPID(&Total_angle[1], &DesiredRatePitch, &DesiredAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, DIRECT);
PID AngleYawPID(&Total_angle[2], &DesiredRateYaw, &DesiredAngleYaw, PAngleYaw, IAngleYaw, DAngleYaw, DIRECT);

PID RateRollPID(&RateRoll, &InputRoll, &ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, DIRECT);
PID RatePitchPID(&RatePitch, &InputPitch, &ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, DIRECT);
PID RateYawPID(&RateYaw, &InputYaw, &ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, DIRECT);

/********* Functions Definition *************/
void gyro_signals(); // của IMU
void neutralPositionAdjustment(); // của Radio Controller
void reset_pid(); // của ESCs (PID)
void SerialDataWrite(); // Cả 3 phần
void channelInterruptHandler(); // Radio Controller 
// A thấy còn function nào chưa define thì ghi ở đây

/********* Clock *************/
unsigned long curr_time = 0, prev_time_com = 0, dt_com = 50000; // time interval in us
unsigned long prev_time_mpu = 0, dt_mpu= 0;
unsigned long time_prev = 0;

void setup(void) {
  Serial.begin(115200); // trong main

/********* Radio Controller *************/
  pinMode(channel_1_pin, INPUT_PULLUP);
  pinMode(channel_2_pin, INPUT_PULLUP);
  pinMode(channel_3_pin, INPUT_PULLUP);
  pinMode(channel_4_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(channel_1_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_2_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_3_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_4_pin), channelInterruptHandler, CHANGE);

/********* ESCs (PID) *************/
  //Motors
  mot1.attach(mot1_pin,1000,2000);
  mot2.attach(mot2_pin,1000,2000);
  mot3.attach(mot3_pin,1000,2000);
  mot4.attach(mot4_pin,1000,2000);
  //to stop esc from beeping
  mot1.write(0);
  mot2.write(0);
  mot3.write(0);
  mot4.write(0);  
  
  AngleRollPID.SetMode(AUTOMATIC);
  AnglePitchPID.SetMode(AUTOMATIC);
  AngleYawPID.SetMode(AUTOMATIC);
  RateRollPID.SetMode(AUTOMATIC);
  RatePitchPID.SetMode(AUTOMATIC);
  RateYawPID.SetMode(AUTOMATIC);

  AngleRollPID.SetOutputLimits(-400, 400);
  AnglePitchPID.SetOutputLimits(-400, 400);
  AngleYawPID.SetOutputLimits(-400, 400);
  RateRollPID.SetOutputLimits(-400, 400);
  RatePitchPID.SetOutputLimits(-400, 400);
  RateYawPID.SetOutputLimits(-400, 400);

  AngleRollPID.SetSampleTime(10);
  AnglePitchPID.SetSampleTime(10);
  AngleYawPID.SetSampleTime(10);
  RateRollPID.SetSampleTime(10);
  RatePitchPID.SetSampleTime(10);
  RateYawPID.SetSampleTime(10);

/********* IMU *************/  
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
  {
    gyro_signals();
    Calib_Roll += RateRoll;
    Calib_Pitch += RatePitch;
    Calib_Yaw += RateYaw;
  }
  Calib_Roll /= 2000;
  Calib_Pitch /= 2000;
  Calib_Yaw /= 2000;
  
  prev_time_mpu = micros();
  prev_time_com = prev_time_mpu;
}

void loop(void) {
  /********* Radio Controller *************/  
  channelInterruptHandler();
  neutralPositionAdjustment();

  /********* ESCs (PID) *************/  
  DesiredAngleRoll=0.1*(ReceiverValue[0]-1500);
  DesiredAnglePitch=0.1*(ReceiverValue[1]-1500);
  InputThrottle=ReceiverValue[2];
  DesiredRateYaw=0.15*(ReceiverValue[3]-1500);

  ErrorAngleRoll= DesiredAngleRoll-Total_angle[0];
  ErrorAnglePitch= DesiredAnglePitch-Total_angle[1];
  ErrorAngleYaw= DesiredAngleYaw-Total_angle[2];

  /********* IMU *************/
  gyro_signals();
  RateRoll -= Calib_Roll;
  RatePitch -= Calib_Pitch;
  RateYaw -= Calib_Yaw;
  dt_mpu = (curr_time - prev_time_mpu);
  prev_time_mpu = curr_time;

  Acc_angle[0] = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * r_to_d;
  Acc_angle[1] = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * r_to_d;

  Gyro_angle[0] += RateRoll * dt_mpu / 1000000;
  Gyro_angle[1] += RatePitch * dt_mpu / 1000000;
  Gyro_angle[2] += RateYaw * dt_mpu / 1000000;

  Total_angle[0] = 0.98 * (Total_angle[0] + RateRoll * dt_mpu / 1000000) + 0.02 * Acc_angle[0];
  Total_angle[1] = 0.98 * (Total_angle[1] + RatePitch * dt_mpu / 1000000) + 0.02 * Acc_angle[1];
  Total_angle[2] = Gyro_angle[2];
  
  /********* ESCs (PID) *************/
  AngleRollPID.Compute();
  AnglePitchPID.Compute();
  AngleYawPID.Compute();

  ErrorRateRoll=DesiredRateRoll-RateRoll;
  ErrorRatePitch=DesiredRatePitch-RatePitch;
  ErrorRateYaw=DesiredRateYaw-RateYaw;

  RateRollPID.Compute();
  RatePitchPID.Compute();
  RateYawPID.Compute();

  if (InputThrottle > 1800) { InputThrottle = 1800; }

  // MotorInput1 =  (InputThrottle + InputRoll - InputPitch + InputYaw); //front left - clockwise  
  // MotorInput2 =  (InputThrottle - InputRoll - InputPitch - InputYaw); // front right - counter clockwise
  // MotorInput3 =  (InputThrottle - InputRoll + InputPitch + InputYaw); // rear right - clockwise
  // MotorInput4 =  (InputThrottle + InputRoll + InputPitch - InputYaw); // rear left  - counter clockwise

  MotorInput1 = InputThrottle - InputRoll + InputPitch + InputYaw; // Front left - counterclockwise
  MotorInput2 = InputThrottle - InputRoll - InputPitch - InputYaw; // Front right - clockwise
  MotorInput3 = InputThrottle + InputRoll - InputPitch + InputYaw; // Rear right - counterclockwise
  MotorInput4 = InputThrottle + InputRoll + InputPitch - InputYaw; // Rear left  - clockwise

  if (MotorInput1 > 2000) { MotorInput1 = 1950; }
  if (MotorInput2 > 2000) { MotorInput2 = 1950; }
  if (MotorInput3 > 2000) { MotorInput3 = 1950; }
  if (MotorInput4 > 2000) { MotorInput4 = 1950; }

  int ThrottleIdle = 1150;
  if (MotorInput1 < ThrottleIdle) { MotorInput1 = ThrottleIdle; }
  if (MotorInput2 < ThrottleIdle) { MotorInput2 = ThrottleIdle; }
  if (MotorInput3 < ThrottleIdle) { MotorInput3 = ThrottleIdle; }
  if (MotorInput4 < ThrottleIdle) { MotorInput4 = ThrottleIdle; }

  int ThrottleCutOff = 1000;
  if (ReceiverValue[2] < 1080)
  {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    reset_pid();
  }

  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);

  curr_time = micros();
  if (curr_time - time_prev >= 20000)
  {
    time_prev = micros();
    SerialDataWrite();
  }
}

/********* Cả 3 *************/
void SerialDataWrite()
{
/********* Radio Controller *************/
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
  Serial.print(ReceiverValue[3]);
  Serial.print(" --- ");

/********* ESCs (PID) *************/ 
  Serial.print("Mot1: ");
  Serial.print(MotorInput1);
  Serial.print(" - ");
  Serial.print("Mot2: ");
  Serial.print(MotorInput2);
  Serial.print(" - ");
  Serial.print("Mot3: ");
  Serial.print(MotorInput3);
  Serial.print(" - ");
  Serial.print("Mot4: ");
  Serial.print(MotorInput4);
  Serial.print(" --- ");

//ESCs (PID)
// //Reciever translated rates
//   Serial.print(DesiredRateRoll);
//   Serial.print("  ");
//   Serial.print(DesiredRatePitch);
//   Serial.print("  ");
//   Serial.print(DesiredRateYaw);
//   Serial.print(" -- ");

//PID outputs
// Serial.print("PID O/P ");
// Serial.print(InputPitch);
//   Serial.print("  ");
// Serial.print(InputRoll);
//   Serial.print("  ");
// Serial.print(InputYaw);
//   Serial.print(" -- ");

/********* IMU *************/
// //Gyro Rates
  // Serial.print(" Gyro rates:");
  // Serial.print(RateRoll);
  // Serial.print("  ");
  // Serial.print(RatePitch);
  // Serial.print("  ");
  // Serial.print(RateYaw);
  // Serial.print(" -- ");

//Angles from MPU
  Serial.print("Roll: ");
  Serial.print(Total_angle[0]);
  Serial.print(" - ");
  Serial.print("Pitch: ");
  Serial.println(Total_angle[1]);
  Serial.print("Yaw: ");
  Serial.println(Total_angle[2]);
  Serial.println(" "); 
}

/********* Radio Controller *************/
void channelInterruptHandler()
{
  current_time = micros();
  // Channel 1
  if (digitalRead(channel_1_pin))
  {
    if (last_channel_1 == 0)
    {
      last_channel_1 = 1;
      timer_1 = current_time;
    }
  }
  else if (last_channel_1 == 1)
  {
    last_channel_1 = 0;
    ReceiverValue[0] = current_time - timer_1;
  }

  // Channel 2
  if (digitalRead(channel_2_pin))
  {
    if (last_channel_2 == 0)
    {
      last_channel_2 = 1;
      timer_2 = current_time;
    }
  }
  else if (last_channel_2 == 1)
  {
    last_channel_2 = 0;
    ReceiverValue[1] = current_time - timer_2;
  }

  // Channel 3
  if (digitalRead(channel_3_pin))
  {
    if (last_channel_3 == 0)
    {
      last_channel_3 = 1;
      timer_3 = current_time;
    }
  }
  else if (last_channel_3 == 1)
  {
    last_channel_3 = 0;
    ReceiverValue[2] = current_time - timer_3;
  }

  // Channel 4
  if (digitalRead(channel_4_pin))
  {
    if (last_channel_4 == 0)
    {
      last_channel_4 = 1;
      timer_4 = current_time;
    }
  }
  else if (last_channel_4 == 1)
  {
    last_channel_4 = 0;
    ReceiverValue[3] = current_time - timer_4;
  }
}

/********* Radio Controller *************/
void neutralPositionAdjustment()
{
  int min = 1490;
  int max = 1510;
  if (ReceiverValue[0] < max && ReceiverValue[0] > min)
  {
    ReceiverValue[0]= 1500;
  } 
  if (ReceiverValue[1] < max && ReceiverValue[1] > min)
  {
    ReceiverValue[1]= 1500;
  } 
  if (ReceiverValue[3] < max && ReceiverValue[3] > min)
  {
    ReceiverValue[3]= 1500;
  } 
  if(ReceiverValue[0]==ReceiverValue[1] && ReceiverValue[1]==ReceiverValue[3] && ReceiverValue[3]==ReceiverValue[0] )
  {
    ReceiverValue[0]= 1500;
    ReceiverValue[1]= 1500;
    ReceiverValue[3]= 1500;
  }

}

/********* IMU *************/
void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll = (float)GyroX/131.0;
  RatePitch = (float)GyroY/131.0;
  RateYaw = (float)GyroZ/131.0;
  AccX = (float)AccXLSB/16384.0;
  AccY = (float)AccYLSB/16384.0;
  AccZ = (float)AccZLSB/16384.0;
//   Acc_angle[0] = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*r_to_d;
//   Acc_angle[1] = -atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*r_to_d;
}
// void gyro_signals(void)
// {
//   Wire.beginTransmission(0x68);
//   Wire.write(0x1B); 
//   Wire.write(0x8);
//   Wire.endTransmission();                                                   
//   Wire.beginTransmission(0x68);
//   Wire.write(0x43);
//   Wire.endTransmission();
//   Wire.requestFrom(0x68,6);
//     GyroX = Wire.read() << 8 | Wire.read();
//     GyroY = Wire.read() << 8 | Wire.read();
//     GyroZ = Wire.read() << 8 | Wire.read();
//     RateRoll = (float)GyroX / 131.0;
//     RatePitch = (float)GyroY / 131.0;
//     RateYaw = (float)GyroZ / 131.0;
// }
// void acc_signals(void)
// {
//     Wire.beginTransmission(0x68);
//     Wire.write(0x1A);
//     Wire.write(0x05);
//     Wire.endTransmission();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x1C);
//     Wire.write(0x10);
//     Wire.endTransmission();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x3B);
//     Wire.endTransmission(); 
//     Wire.requestFrom(0x68,6);
//     AccXLSB = Wire.read() << 8 | Wire.read();
//     AccYLSB = Wire.read() << 8 | Wire.read();
//     AccZLSB = Wire.read() << 8 | Wire.read();
//     AccX = (float)AccXLSB / 16384.0;
//     AccY = (float)AccYLSB / 16384.0;
//     AccZ = (float)AccZLSB / 16384.0;
// }

/********* ESCs (PID) *************/
void reset_pid(void)
{
  // Total_angle[0] = 0; Total_angle[1]=0; Total_angle[2] = 0;
  DesiredRateRoll = 0; DesiredRatePitch = 0; DesiredRateYaw = 0;
  DesiredAngleRoll = 0; DesiredAnglePitch= 0;  DesiredAngleYaw = 0;  
  InputRoll = 0; InputPitch = 0; InputYaw = 0;
}