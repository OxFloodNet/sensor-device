/** 
 * OxFloodNet_Sensor sketch for Ciseco RFu-328 and Maxbotix MB7060 XL-MaxSonar-WR1, Standard edition ultrasonic sensor
 *
 * Transmits reading every 1, 5, 10 or 15 minutes depending on jumpers. Default is 15 minutes
 * Data transmitted is of form aXXUnnn-----
 * Battery readings every 10th cycle  aXXBvvvv----
 *
 * Where XX is the device ID, nnn is the distance to the water in cm, vvvv is battery voltage in mV
 * Other messages sent are: aXXSTARTvvv- upon startup, vvv is version ID
 * aXXUMax----- and aXXUErr----- if max range is reached or an error in the reading is seen, i.e. too close.
 * aXXUnnnn----, aXXDnnnn---- and aXXTnn.n----
 * D is compensated distance, U is Raw Distance, T is temperature and B is battery reading
 *
 * Sensor is controled using a single N-Channel MOSFET, Gate to SENSOR_ENABLE,
 * Source to GND, Drain to -ve of sensor, Sensor +ve to +3V.
 *
 * For first 10 minutes after reset/powerup, sensor wakeup is every twenty seconds (
 * for next 10 minutes the sleep is every 1 minutes, after this sleep interval
 * becomes value set on jumpers (5, 10 or 15 mins) 
 *

 * (c) 2014 Andrew D Lindsay & Ben Ward 
 */

#include "LLAPSerial.h"	// include the library
#include <OneWire.h>
#include <DallasTemperature.h>

// Used in START msg to indicate firmware version, 315 = 3.1.5
#define VERSION_ID 315

// Set RFu hardware pins (not board jumpers)
// Enable SRF
#define SRF_RADIO_ENABLE 8
// Enable the sensor, controlled by FET
#define SENSOR_ENABLE 6
// Pin that sensor PWM input is on
#define SENSOR_PIN 3
// Using Sleep Mode 1, SLEEP must be HIGH to run
// Using Sleep Mode 2, SLEEP must be LOW to run
// Using Sleep Mode 3, SLEEP must be LOW to run, uses interrupt to wakeup
#define SRF_SLEEP 4
#define WAKE_INT 2

// Temperature sensor data line connects to pin 5 on the Arduino
// OneWire bus pin for temperature sensor pin
#define ONE_WIRE_BUS 5

// Software configuration defines
// Number of readings before a battery reading is taken
#define BATTERY_READ_INTERVAL 10

// Polling interval jumper link enumeration values, 1, 5, 10 or 15 minutes
#define FIFTEENMINS 0
#define TENMINS 1
#define FIVEMINS 2
#define ONEMIN 3
#define TWENTYSEC 4

// Define AT commands to set polling intervals defined above
// Order is Fifteen Mins, Ten Mins, Five Mins, 1 Min, 20 seconds
char *pollingIntervalCmds[] = { 
  "ATSDDBBA0",  // 15 Minutes
  "ATSD927C0",  // 10 Minutes
  "ATSD493E0",  // 5 Minutes
  "ATSDEA60",   // 1 Minute
  "ATSD4E20"    // 20s
};

// define startup polling modes
// 10 minutes of 20s interval is 30 wakeups
// After 10 mins this is reset to 10 and interval is 1 minute 
#define NORMAL_POLL 0
#define MED_POLL 1
#define FAST_POLL 2

// Counts for the startup polling modes
#define FAST_POLL_COUNT 30
#define MED_POLL_COUNT 10

// Hardware objects

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature library.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress temperatureSensor;

// Define pins that are not used and are to be made inputs for power saving
// TODO: Check usage of all pins
#define PIN_COUNT 5

uint8_t inPin[] = { 
  9, 10, 11, 12, 13 };

// Other global variables
boolean tempSensorFound = false;
int batteryCountDown = BATTERY_READ_INTERVAL;
int pollingInterval = FIFTEENMINS;
int startupSequence = FAST_POLL;
int startupCounter = FAST_POLL_COUNT;

// Node ID, default 00, set by input pins
// Strictly speaking this should only be digits, according to the Ciseco LLAP spec.
// but since the spec is inefficient we've ignored the format.
char nodeId[2] = {  '0', '0' };


// Some functions to get the configured node address and polling
/** readJumpers - determine node ID and polling interval for this sensor
 */
void readJumpers() {
  // Set analog input pins to read digital values, set internal pullup
  uint8_t jumperPins[] = { 
    A0, A1, A2, A3, A4, A5, 9, 10, 12, 13   };
  for( int n = 0; n < 10; n++ ) {
    pinMode( jumperPins[n], INPUT );
    digitalWrite( jumperPins[n], HIGH);
  }

  // Set individual registers
  // TODO: Explain how these relate to the board
  // TODO: These are upside down for the board layout.
  int digit2 = ((PINB & 0x30) >> 4) | ((PINC & 0x03) << 2);
  digit2 ^= 0x0f;
  int digit1 = (PINC & 0x3c) >> 2;
  digit1 ^= 0x0f;

  // Set Node ID from jumper pins
  // Only use digits 0 - 9, 
  nodeId[0] = '0' + min(digit1,9);
  nodeId[1] = '0' + min(digit2,9);

  // Set polling interval from jumper pins
  pollingInterval = ((PINB & 0x06) >> 1) ^ 0x03;

  // Reset inputs to set internal pullup off
  for( int n = 0; n < 10; n++ ) {
    digitalWrite( jumperPins[n], LOW);
  }

}

// Functions used in processing readings

/* isort - Simple sort function for the set of readings
 * Sorting function (Author: Bill Gentles, Nov. 12, 2010)
 * The sorted array is returned in the original array
 * @param a - Array of unsigned 16 bit values to sort
 * @param n - Number of values in array
 */
void isort(uint16_t *a, int8_t n){
  for (int i = 1; i < n; ++i)  {
    uint16_t j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--) {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }
}


/** mode - Mode function, returning the mode or median.
 * @param x - Array of unsigned 16 bit values to sort
 * @param n - Number of values in array
 * @return mode or median of the passed data.
 */
uint16_t mode(uint16_t *x,int n){
  int i = 0;
  int count = 0;
  int maxCount = 0;
  uint16_t mode = 0;
  int bimodal;
  int prevCount = 0;
  while(i<(n-1)){
    prevCount=count;
    count=0;
    while( x[i]==x[i+1] ) {
      count++;
      i++;
    }
    if( count > prevCount & count > maxCount) {
      mode=x[i];
      maxCount=count;
      bimodal=0;
    }
    if( count == 0 ) {
      i++;
    }
    if( count == maxCount ) {      //If the dataset has 2 or more modes.
      bimodal=1;
    }
    if( mode==0 || bimodal==1 ) {  // Return the median if there is no mode.
      mode=x[(n/2)];
    }
    return mode;
  }
}

// Start of Ultrasonic sensor code

/** getrange - Get the range in cm. Need to enable sensor first, waiting briefly for it to power up.
 * Request temperature sensor, if previously detected, to take a reading.
 * Disable ultrasonic sensor after use for power saving.
 * Returns compensated, uncompensated and temperature values by reference.
 * Calling function can still do checks to determine whether to use reading or not.
 *
 * @param outRawDistance - Raw uncompensated distance
 * @param outDistance - Compensated distance if temperature available, otherwise raw distance
 * @param outTemperature - Temperature reading
 * @return boolean true returned form function for valid reading. false for <23cm or out of range
 */
boolean getRange( int *outRawDistance, int *outDistance, float *outTemperature ) {

  int16_t pulse;  // number of pulses from sensor
  int i=0;
  // These values are for calculating a mathematical median for a number of samples as
  // suggested by Maxbotix instead of a mathematical average
  int8_t arraysize = 9; // quantity of values to find the median (sample size). Needs to be an odd number
  //declare an array to store the samples. not necessary to zero the array values here, it just makes the code clearer
  //  uint16_t rangevalue[] = { 
  //    0, 0, 0, 0, 0, 0, 0, 0, 0 };
  uint16_t pulsevalue[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0   };

  float temperature = 0.0;
  digitalWrite(SENSOR_ENABLE, HIGH);
  delay(50);

  while( i < arraysize ) {								    
    pulse = pulseIn(SENSOR_PIN, HIGH);  // read in time for pin to transition
    if( pulse == 0 ) return 0;
    //    rangevalue[i]=pulse/58;         // pulses to centimeters (use 147 for inches)
    pulsevalue[i]=pulse;              // raw pulses to centimeters (use 147 for inches)
    //    if( rangevalue[i] < 645 && rangevalue[i] >= 15 ) i++;  // ensure no values out of range
    if( pulsevalue[i] < 37410 && pulsevalue[i] >= 870 ) i++;  // ensure no values out of range
    delay(10);                      // wait between samples
  }

  // Turn off sensor as no longer needed
  digitalWrite(SENSOR_ENABLE, LOW);

  //  isort(rangevalue,arraysize);        // sort samples
  //  uint16_t distance = mode(rangevalue,arraysize);  // get median 
  isort(pulsevalue,arraysize);        // sort samples
  uint16_t rawtof = mode(pulsevalue,arraysize);  // get median 

  //  *outRawDistance = distance;
  *outRawDistance = rawtof/58;      // time of flight / 58uS for both there and back
  // Use temperature compensation if temp sensor found
  if( tempSensorFound ) {
    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0);
    *outTemperature = temperature;
    //    float tof = distance * 0.58;
    //    uint16_t newDist = tof * (( 20.05 * sqrt( temperature + 273.15))/200);
    uint16_t newDist = (rawtof * (331.3 + 0.606 * temperature)) / 20000;

    *outDistance = newDist;
  } else {
    // No temperature sensor, just return same distance for both
    *outDistance = rawtof/58;  
  }
  
  // Add check for validity of reading
  if( *outRawDistance <= 23 ) {    // 23cm seems to be the minimum value
    return false;
  }

  return true; 
}


// End of ultrasonic sensor code
// **************************************

// Battery monitoring

/** readVcc - Read the current battery voltage
 * @return Vcc in millivolts
 */
int readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
//  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
//  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return (int)result; // Vcc in millivolts
}
// end of battery monitoring code




// Start of SRF Sleep Code
/** setSRFSleep - Set the sleep mode on the SRF, this requires AT commands to set sleep interval
 * This is not written to permanent store as this resets back to no sleep on power cycle.
 * Set sleep based on current startup sequence and polling interval 
 * @param inPolling - The Polling parameter 
 * @return success code, 5 for success, other values indicate where failure occured
 */


uint8_t setSRFSleep( int inPolling ) 
{
  if (!enterCommandMode())	// if failed once then try again
  {
    if (!enterCommandMode()) 
      if (!enterCommandMode()) 
        return 1;
  }

  // setup the selected polling interval
  if (!sendCommand( pollingIntervalCmds[inPolling]) ) return 2;

  if (!sendCommand("ATSM3")) return 3;
  if (!sendCommand("ATDN")) return 4;
  return 5; // success
}

// end of SRF sleep code

// Start of SRF command functions

/** enterCommandMode - Enter command mode on the SRF radio module. From here
 * AT commands can be sent to reconfigure it.
 * @return status to indicate if the SRF is in command mode now
 */
uint8_t enterCommandMode()
{
  delay(1200);
  Serial.print("+++");
  delay(500);
  while (Serial.available()) Serial.read();  // flush serial in - get rid of anything received before the +++ was accepted
  delay(500);
  return checkOK(500);
}

/** sendCommand - Send the AT command to the SRD module.
 * @param lpszCommand - Char pointer to the command string
 * @return status to indicate command was sent.
 */
uint8_t sendCommand(char* lpszCommand)
{
  Serial.print(lpszCommand);
  Serial.write('\r');
  return checkOK(100);
}

/** checkOK - wait a specified time for the OK prompt to be returned after sending a 
 * command to the SRF module.
 * @param  timeout - The timeout in milliseconds to wait before giving up
 * @return 1 for successfully seen the OK, 0 for timeout
 */
uint8_t checkOK(uint32_t timeout)
{
  uint32_t time = millis();
  while (millis() - time < timeout)
  {
    if (Serial.available() >= 3)
    {
      if (Serial.read() != 'O') continue;
      if (Serial.read() != 'K') continue;
      if (Serial.read() != '\r') continue;
      return 1;
    }
  }
  return 0;
}
// End of SRF command functions

/** setup - Regular Arduino setup function. 
 * We setup the I/O pins, read jumpers and start the sensor
 */
void setup() {
  // initialise serial interface for SRF communication:
  Serial.begin(115200);

  // Get device ID
  readJumpers();

  // Initialise the LLAPSerial library
  LLAP.init( nodeId );
  // ?
  analogReference(DEFAULT);

  // Set unused digital pins to input and turn on pull-up resistor
  for(int i = 0; i< PIN_COUNT; i++ ) {
    pinMode(inPin[i], INPUT);
    digitalWrite(inPin[i], HIGH);
  }

  // Setup sensor pins
  pinMode(SENSOR_ENABLE, OUTPUT);
  digitalWrite(SENSOR_ENABLE, LOW);
  pinMode(SENSOR_PIN, INPUT);

  // Setup the SRF pins
  pinMode(SRF_RADIO_ENABLE, OUTPUT);    // initialize pin to control the radio
  digitalWrite(SRF_RADIO_ENABLE, HIGH); // select the radio
  pinMode(SRF_SLEEP, OUTPUT);
  digitalWrite( SRF_SLEEP, LOW );

  // start OneWire sensors
  sensors.begin();
  tempSensorFound = false;				//clear flag for sensor discovery
  // Discover OneWire temperature sensor
  if (sensors.getAddress(temperatureSensor, 0)) {
    // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)

    sensors.setResolution(temperatureSensor, 9);
    tempSensorFound = true; // set discovery flag
  }

  // Initialise countdown number of sleeps before sending battery reading
  batteryCountDown = BATTERY_READ_INTERVAL;
  // Wait for it to be initialised
  delay(200);

  // Send "START" message if successful or "ERR" if not able to set SleepMode 3.
  // set up sleep mode 3 (low = awake)
  uint8_t val;
  while ((val = setSRFSleep(TWENTYSEC)) != 5) {
    LLAP.sendInt("ERR",val); // Diagnostic
    delay(5000);	// try again in 5 seconds
  }


  // Send "START" message and Firmware version 5 times
  for(int i = 0; i<5; i++) {
    LLAP.sendInt("START", VERSION_ID);
    // TODO: Wait 100mS for ACK, if received continue, otherwise send next STARTED
    delay(20);
  }
}

/** loop - where the sensor code runs. 
 * we wake up the SRF, take a reading, transmit reading then go back to sleep
 */
void loop() {
  float temperature = 0.0;
  int rawDistance = 0;
  int distance = 0;

  // Determine if we need to send a battery voltage reading or a distance reading
  if( --batteryCountDown <= 0 ) {
    int mV = readVcc();
    LLAP.sendIntWithDP("B", mV, 3 );
    batteryCountDown = BATTERY_READ_INTERVAL;
    delay(20);
  } 

  // Distance reading
  boolean rangeValid = getRange( &rawDistance, &distance, &temperature);
  // Send temperature reading
  if( tempSensorFound ) {
    int latestTemp = (int)(temperature * 100);
    LLAP.sendIntWithDP( "T", latestTemp, 2);

    delay(20);
  }
  // Uncompensated distance
  LLAP.sendInt( "U", rawDistance);
  delay(20);
  
  // TODO: Send reading 3 times to make sure it gets through


  if( rangeValid ) {
    //if( rawDistance > 23 ) {    // 23cm seems to be the minimum value
    LLAP.sendInt( "D", distance );
  } 
  else {
    if( rawDistance == 0 ) {
      LLAP.sendMessage( "UMax" );
    } 
    else {
      LLAP.sendMessage( "UErr" );

    }

  }

  // Determine if we are still in startup sequence, if so, do we need to
  // adjust polling interval.
  if( startupSequence != NORMAL_POLL ) {
    if( --startupCounter <= 0 ) {
      // Move to next sequence
      startupSequence--;
      // Reset time interval
      int selectedPollingInterval = TENMINS;
      // Setup for next polling sequence
      switch( startupSequence ) {
        // removed as we would never use this code        

        //      case FAST_POLL:
        //        selectedPollingInterval = TWENTYSEC;
        //        break;
      case MED_POLL:
        selectedPollingInterval = ONEMIN;
        startupCounter = MED_POLL_COUNT;
        break;
      default:
        // Use value set by jumpers
        selectedPollingInterval = pollingInterval;
        break;
      }  
      delay(50);
      uint8_t val;
      while ((val = setSRFSleep(selectedPollingInterval)) != 5) {
        LLAP.sendInt("ERR",val); // Diagnostic
        delay(5000);	// try again in 5 seconds
      }
    }
  }
  // Short delay to allow reading to be sent then put sensor to sleep 
  delay(50);
  pinMode(SRF_SLEEP, INPUT);                // sleep the radio
  LLAP.sleep(WAKE_INT, RISING, false);      // sleep until woken on pin 2, no pullup (low power)
  pinMode(SRF_SLEEP, OUTPUT);               // wake the radio
}
// That's all folks






