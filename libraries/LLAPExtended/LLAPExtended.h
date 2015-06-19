#ifndef _LLAPEXTENDED_h
#define _LLAPEXTENDED_h

#if defined(ARDUINO) && ARDUINO >= 100
        #include "Arduino.h"
#else
        #include "WProgram.h"
#endif

class LLAPExtended
{
 private:
        //char cMessage[23];
        void processMessage();
 public:
        char cMessage[19];
        void init();
        void init(char* cI);
        char deviceId[8];
        String sMessage;
        boolean bMsgReceived;
        void SerialEvent();
    void sendMessage(String sToSend);
        void sendMessage(char* sToSend);
        void sendMessage(char* sToSend, char* valueToSend);
        void sendMessage(const __FlashStringHelper *ifsh);
        void sendMessage(const __FlashStringHelper *ifsh, char* valueToSend);
        void sendInt(String sToSend, int value);
        void sendIntWithDP(String sToSend, int value, byte decimalPlaces);
    void setDeviceId(char* cId);
        byte sleepForaWhile (long msecs);        // timed sleep using the watchdog
        void sleep(byte pinToWakeOn, byte direction = FALLING, byte bPullup = true);                // full sleep woken by pin interrupt
};

extern LLAPExtended LLAP;

#endif

