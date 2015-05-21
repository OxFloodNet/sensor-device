/** 
 * OxFloodNet_Sensor sketch for Ciseco RFu-328 and Maxbotix MB7060 XL-MaxSonar-WR1, Standard edition ultrasonic sensor
 * Test mode functions.
 *
 * To access te sensor test mode you need a communicate with the sensor using a SRF stick plugged into
 * a laptop/desktop/tablet with a suitable terminal emulator, e.g. TeraTerm for Windows. The SRF stick needs to
 * be configured to operate on an alternative PANID so that it does not interfere with normal sensor readings.
 * To configure, use the following AT commands to setup the SRF Stick:
 *
 * +++
 * ATID2305
 * ATPKF0
 * ATAC
 * ATDN
 *
 * The sensor should have a jumper link placed on the 'Test' pins, then once the sensor has been reset by
 * stroking a magnet along the side closes to the battery, you should see a welcome message and menu listing
 * the available options. To return the sensor to normal operation you need to remove the 'Test' jumper and reset
 * the sensor.
 *
 * The test options allow you to view the sensor node ID from hte DS18B20 temperature sensor, see the
 * temperature readings and distance readings.
 *
 *
 * (c) 2015 Andrew D Lindsay & Ben Ward for Oxford Flood Network
 * http://oxfloodnet.co.uk/ 
 * @oxfloodnet 
 *
 */

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Enable SRF
#define SRF_RADIO_ENABLE 8

// External functions from main sketch
extern uint8_t enterCommandMode();
extern uint8_t sendCommand(char*);
extern uint8_t checkOK(uint32_t);
extern boolean getRange( int *, int *, float * );
extern int readVcc();

// External variable declarations
extern OneWire oneWire;
extern DallasTemperature sensors;
extern DeviceAddress temperatureSensor;
extern uint8_t pollingInterval;
extern uint16_t fwVersion;

// Function declarations for use in menu
void dummyOption();
void displayAddress();
void readTemperature();
void readRawDistance();
void readDistance();
void readBatteryVoltage();
void readPollingRate();
void printAddress(DeviceAddress);

// Define our menu, number of items then list of menu options and functions to call
#define NUM_MENU_ITEM 7
// Maximum length of a menu option text is 17 characters, 18 used to include termination character
char menu_items[NUM_MENU_ITEM][18]={
  "Display Menu",
  "Display Address",
  "Read Temperature",
  "Read Raw Distance",
  "Read Distance",
  "Battery Voltage",
  "Read Polling Rate"
};

// Array of functions. This makes coding the menu actions easy and more options can be added without code changes.
void (*menu_funcs[NUM_MENU_ITEM])(void) = {
  dummyOption,
  displayAddress,
  readTemperature,
  readRawDistance,
  readDistance,
  readBatteryVoltage,
  readPollingRate
};

/** setSRFTestMode - Set the SRF for test mode, use alternative PANID and packet size.
 * This is not written to permanent store as this resets back to defaults on power cycle or reset.
 * @return success code, 0 for success, other values indicate where failure occured
 */
uint8_t setSRFTestMode() {
  if (!enterCommandMode())	// if failed once then try again
  {
    if (!enterCommandMode()) 
      if (!enterCommandMode()) 
        return 1;
  }

  if (!sendCommand("ATID2305") ) return 2;    // PANID 2305
  if (!sendCommand("ATPK40")) return 3;       // 64 bytes
  if (!sendCommand("ATMYOX")) return 4;       // ID for Remote programming
  if (!sendCommand("ATAC")) return 5;         // Action config
  if (!sendCommand("ATDN")) return 6;         // Exit AT mode
  return 0; // success
}

// Menu functions
/** isKeypress - Check is we've received a keypress or character over the serial link.
 * @return True for a keypress, false for no keypress
 */
boolean isKeypress() {
  if (Serial.available()) {
    int inByte = Serial.read();    // Consume character
    return true;
  }
  return false;
}

/** dummyOption - Dummy, empty function just for option 0
 */
void dummyOption() {
}

/** readAddress - Menu option to display the address of the sensor
 */
void displayAddress() {
  Serial.print(F("\n\rAddress: "));
  printAddress(temperatureSensor);
  Serial.println();
}

/** printAddress - Print the address retrieved form the temperature sensor
 */
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 1; i < 5; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

/** readTemperature - Read the temperature from the temperature sensor and display.
 * repeatedly read and display the reading with a 500mS delay until a key is pressed.
 */
void readTemperature() {
  Serial.println(F("\n\rRead Temperature"));
  float temperature = 0.0;
  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  int resolution = 9;
  //    sensors.setResolution(temperatureSensor, resolution);
  while( !isKeypress() ) {
    sensors.requestTemperatures();
    delay(750/ (1 << (12-resolution)));
    Serial.print(F("Temperature: "));
    Serial.print(sensors.getTempC(temperatureSensor));
    Serial.print(F("C     \r"));
    delay(500);
  }
  Serial.println();
}

/** readRawDistance - Read the distance and repeatedly display the uncompensated distance.
 * Delay of 500mS between readings until a key is pressed.
 */
void readRawDistance() {
  Serial.println(F("\n\rRead Raw Distance"));
  float temperature = 0.0;
  int rawDistance = 0;
  int distance = 0;

  while( !isKeypress() ) {
    // Distance reading
    boolean rangeValid = getRange( &rawDistance, &distance, &temperature);
    if( rangeValid ) {
      Serial.print(F("Raw Distance: "));
      Serial.print(rawDistance); 
      Serial.print(F("cm   \r"));
    } 
    else {
      Serial.print(F("Error                  \r"));
    }
    delay(500);
  }
  Serial.println();
}

/** readDistance - Read the distance and repeatedly display the temperature, compensated and uncompensated distances.
 * Delay of 500mS between readings until a key is pressed.
 */
void readDistance() {
  Serial.println(F("\n\rRead Distance"));
  float temperature = 0.0;
  int rawDistance = 0;
  int distance = 0;

  while( !isKeypress() ) {
    // Distance reading
    boolean rangeValid = getRange( &rawDistance, &distance, &temperature);
    if( rangeValid ) {
      Serial.print(F("Temperature: "));
      Serial.print(temperature);
      Serial.print(F("C  Raw Distance: "));
      Serial.print(rawDistance); 
      Serial.print(F("cm  Distance: "));
      Serial.print(distance);
      Serial.print("cm        \r");
    } 
    else {
      Serial.print(F("Error                                                              \r"));
    }
    delay(500);
  }
  Serial.print("\n\r");
}

/* readBatteryVoltage - Read and display the current battery voltage
 */
void readBatteryVoltage() {
  Serial.println(F("\n\rRead Battery Voltage"));
  int mV = readVcc();
  Serial.print(F("Battery: "));
  Serial.print(mV/1000, DEC );
  Serial.print(F("."));
  Serial.print(mV^1000, DEC );
  Serial.println(F("V "));
}

/** readPollingRate - Read and display the current polling rate from the jumpers
 */
void readPollingRate() {
  Serial.println(F("\n\rRead Polling Rate"));
  // Set analog input pins to read digital values, set internal pullup
  uint8_t jumperPins[] = {  
    9, 10     };
  for( int n = 0; n < 2; n++ ) {
    pinMode( jumperPins[n], INPUT );
    digitalWrite( jumperPins[n], HIGH);
  }

  uint8_t pin1 = digitalRead( 10 );
  uint8_t pin2 = digitalRead( 9 );
  // Reset inputs to set internal pullup off
  for( int n = 0; n < 2; n++ ) {
    digitalWrite( jumperPins[n], LOW);
  }
  Serial.print(F("Pin 1 " ));
  Serial.print( pin1 ? "No Jumper" : "Jumper");
  Serial.print(F("\r\nPin 2 "));
  Serial.println( pin2 ? "No Jumper" : "Jumper");

  uint8_t polling = (pin2 << 1 | pin1) ^ 0x03;
  Serial.print(F("Polling: "));
  Serial.println(polling, DEC);
  switch( polling ) {
  case 1:
    Serial.println(F("1 Minute"));
    break;
  case 2:
    Serial.println(F("5 Minutes"));
    break;
  case 3:
    Serial.println(F("10 Minutes"));
    break;
  default:
    Serial.println(F("15 Minutes"));
  }

}

/** displayMenu - Display the menu options and prompt
 */
void displayMenu() {
  Serial.println(F("\n\rTest Menu"));
  Serial.println(F("========="));
  for( int i=0; i< NUM_MENU_ITEM; i++ ) {
    Serial.print(i, DEC);
    Serial.print(F(" - "));
    Serial.println(menu_items[i] );
    Serial.flush();
    delay(20);
  }
  Serial.print(F("Choice: [0 - "));
  Serial.print(NUM_MENU_ITEM-1,DEC);
  Serial.print(F("] "));
  delay(200);
}

/** testMode - main test mode, does setup, displays title, menu and waits for input.
 * Number entered denotes the menu option to call. Use option 0 to redisplay menu.
 */
void testMode() {
  setSRFTestMode();
  digitalWrite(SRF_RADIO_ENABLE, HIGH); // select the radio
  delay(100);    // Allow radio to turn on
  Serial.println(F("\n\r\n\rOxford Flood Network Sensor"));
  Serial.println(F("==========================="));
  Serial.print(F("Firmware Version "));
  Serial.println(fwVersion,DEC);
  displayMenu();

  while(1) {
    if (Serial.available()) {
      int inByte = Serial.read();
      if( inByte >= '0' && inByte <= ('0'+NUM_MENU_ITEM-1)) {
        (*menu_funcs[inByte-'0'])();
        delay(500);
        displayMenu();
      } 
//      else if(inByte == '0' ) {
//        displayMenu();
//      }
    }
  }
}


