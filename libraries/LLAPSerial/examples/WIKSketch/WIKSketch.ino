//
// LLAP - Lightweight Local Automation Protocol
//
// Wireless Inventors Kits
// Ciseco Ltd. Copyright 2013
//

// Changes
//  1.0  Add FVER command
//
#define VERSION "1.0"

#include <EEPROM.h>
#include <LLAPSerial.h>
#include <Servo.h>

#define DEVICETYPE "RASWIK"
#define DEVICEID1 '-'
#define DEVICEID2 '-'
#define EEPROM_DEVICEID1 0
#define EEPROM_DEVICEID2 1

// inputs
uint8_t inputs[] = {2,3,4,7,10,12,254};
// analog
uint8_t analogs[] = {0,1,2,3,4,5,254};
// counter
#define COUNTPIN 4
unsigned int countValue = 0;;
uint8_t countState = 1;
uint32_t countTime;
// interrupt
#define DEBOUNCETIME 150
#define INT1PIN 2
uint8_t int1Value =1;
uint32_t int1Time;
#define INT2PIN 3
uint8_t int2Value =1;
uint32_t int2Time;
// outputs
uint8_t outputs[] = {6,5,11,13,254};
// pwm
uint8_t pwm[] = {6,5,11,254};
// servo
#define SERVOPIN 9
Servo myservo;  // create servo object to control a servo 


String msg;        // storage for incoming message
String reply;    // storage for reply

void setup() // always called at the start to setup I/Os etc
{

  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH); // turn on the radio
  Serial.begin(115200); // start the serial port at 115200 baud

  byte i=0;
  while (inputs[i]) {pinMode(inputs[i],INPUT); digitalWrite(inputs[i++],HIGH);}
  i=0;
  while (outputs[i]) pinMode(outputs[i++],OUTPUT);
  myservo.attach(SERVOPIN);  // attaches the servo pin to the servo object 

  String permittedChars = "-#@?\\*ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  char deviceID[2] = {EEPROM.read(EEPROM_DEVICEID1), EEPROM.read(EEPROM_DEVICEID2)};
  if (permittedChars.indexOf(deviceID[0]) == -1 || permittedChars.indexOf(deviceID[1]) == -1)
  {
	  deviceID[0] = DEVICEID1;
	  deviceID[1] = DEVICEID2;
  }

  LLAP.init(deviceID);

  LLAP.sendMessage("STARTED");
}

void loop() // repeatedly called
{
    if (LLAP.bMsgReceived) // got a message?
    {
        msg = LLAP.sMessage;
        LLAP.bMsgReceived = false;
        reply = msg;
        if (msg.compareTo("HELLO----") == 0)
        {
            ;    // just echo the message back
        }
        else if (msg.compareTo("FVER-----") == 0)
        {
          reply = reply.substring(0,4) + VERSION;
        }
        else if (msg.compareTo("DEVTYPE--") == 0)
        {
          reply = DEVICETYPE;
        }
        else if (msg.compareTo("SAVE-----") == 0)
        {
            EEPROM.write(EEPROM_DEVICEID1, LLAP.deviceId[0]);    // save the device ID
            EEPROM.write(EEPROM_DEVICEID2, LLAP.deviceId[1]);    // save the device ID
        }
        else if (msg.startsWith("SERVO"))
        {
            msg = msg.substring(5);
            int value = msg.toInt();
            if (value >=0 && value <= 180)
                myservo.write(value);
            else
                reply = "TOOLARGE";
        }
        else if (msg.startsWith("COUNT"))
        {
            msg = msg.substring(5);
            if (msg.startsWith("-"))
            {    // read the value
                reply = reply.substring(0,5) + countValue;
            }
            else
            {    // set the value
                int value = msg.toInt();
                if (value >=0 && value <= 9999)
                    countValue = value;
                else
                    reply = "TOOLARGE";
            }
        }
        else    // it is an action message
        {
            byte typeOfIO;
            byte ioNumber;
            typeOfIO = msg.charAt(0);
            ioNumber = (msg.charAt(1) - '0') * 10 + msg.charAt(2) - '0';
            msg = msg.substring(3);
            if (msg.compareTo("HIGH--") == 0)
            {
                if (validPin(outputs,ioNumber))
                    digitalWrite(ioNumber,HIGH);
                else
                    reply = "NOTOUTPUT";
            }
            else if (msg.compareTo("LOW---") == 0)
            {
                if (validPin(outputs,ioNumber))
                    digitalWrite(ioNumber,LOW);
                else
                    reply = "NOTOUTPUT";

            }
            else if (msg.startsWith("PWM"))
            {
                byte val = ((msg.charAt(3) - '0') * 10 + msg.charAt(4) - '0') * 10 + msg.charAt(5) - '0';
                if (val >=0 && val <= 255)
                {
                    if (validPin(pwm,ioNumber))
                      analogWrite(ioNumber,val);
                    else
                      reply = "NOTPWM";
                }
                else
                    reply = "TOOLARGE";
              
            }
            else if (msg.compareTo("READ--") == 0)
            {
                reply = reply.substring(0,3);
                if (typeOfIO == 'A')
                {
                    if (validPin(analogs,ioNumber))
                    {
                        int val = analogRead(ioNumber);
                        reply = reply + "+" + val;
                    }
                    else
                        reply = "NOTINPUT";
                }
                else
                {
                    if (validPin(inputs,ioNumber))
                    {
                        byte val = digitalRead(ioNumber);
                        if (val)
                        {
                            reply = reply + "HIGH";
                        }
                        else
                        {
                            reply = reply + "LOW";
                        }
                    }
                    else
                        reply = "NOTINPUT";
                }
            }
            else
                reply = "ERROR";
        }
        LLAP.sendMessage(reply);
    }
    else
    {
        checkForInterruptPin(INT1PIN, &int1Value, &int1Time);
        checkForInterruptPin(INT2PIN, &int2Value, &int2Time);
        // and increment counter if needed
        if (millis() - countTime > DEBOUNCETIME)
        {
            if (countState && digitalRead(COUNTPIN) == 0)
            {
                countState = 0;
                countValue++;
                if (countValue > 9999) countValue = 0;
            }
            else if (countState == 0 && digitalRead(COUNTPIN))
            {
                countState = 1;
            }
            countTime = millis();
        }
    }
}

boolean validPin(byte* pins, byte pinNumber)
{
    byte i = 0;
    while (pins[i] != 254)  // end of array check numebr
    {
        if (pins[i++] == pinNumber)
            return true;
    }
    return false;
}

void checkForInterruptPin(uint8_t intpin, uint8_t* pinvalue, uint32_t* pinTime)
{
    // check interrupts
    // crude debounce
    // once changed don't look for another change until DEBOUNCETIME has elapsed
    if (digitalRead(intpin) != *pinvalue && millis() - *pinTime > DEBOUNCETIME)
    {    
        *pinvalue = digitalRead(intpin);
        *pinTime = millis();
        reply = "D0";
        reply += intpin;
        if (*pinvalue)
        {
            reply = reply + "HIGH";
        }
        else
        {
            reply = reply + "LOW";
        }
        LLAP.sendMessage(reply);
    }
}
