#ifndef PROTO1_RADIO_CTRL_H
#define PROTO1_RADIO_CTRL_H

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

class Proto1_RadioCtrl {
public:
    Proto1_RadioCtrl();
    void neutralPosition();
    void attachInterrupts();
    void updateChannelValues(int channel);
    const volatile int* getChannelValues() const; // Accessor for channel values
    void printChannelStatus();

    static Proto1_RadioCtrl* instance; // Static pointer to the instance used in ISRs

private:
    volatile uint32_t timers[4];
    volatile int receiverValues[4];
    volatile int last_channels[4];
    int channel_pins[4] = {19, 18, 17, 16};
    //int channel_pins[4] = {13, 14, 27, 26};
    static void ISRWrapper0();
    static void ISRWrapper1();
    static void ISRWrapper2();
    static void ISRWrapper3();
};

#endif
