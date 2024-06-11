#ifndef RC_H
#define RC_H

extern double ReceiverValue[4];

// Channel pin constants
const int channel_1_pin = 19;
const int channel_2_pin = 18;
const int channel_3_pin = 17;
const int channel_4_pin = 16;

// Function Declarations
void channelInterruptHandler();
void neutralPositionAdjustment();
void setupRC();
void SerialDataPrintRC();

#endif // RC_H