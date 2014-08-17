/** 
 * LLAPDistance_Remote sketch for Ciseco RFu-328 and Maxbotix MB7060 XL-MaxSonar-WR1, Standard edition ultrasonic sensor
 *
 * Transmits reading every 30s, 5, 10 or 15 minutes depending on jumpers. Default is 10 minutes
 * Data transmitted is of form aXXUnnn-----
 * Battery readings every 10th cycle  aXXBvvvv----
 *
 * Where XX is the device ID, nnn is the distance to the water in cm, vvvv is battery voltage in mV
 * Other messages sent are: aXXSTARTED upon startup
 * aXXUMax----- and aXXUErr----- if max range is reached or an error in the reading is seen, i.e. too close.
 *
 * Sensor is controled using a single N-Channel MOSFET, Gate to SENSOR_ENABLE,
 * Source to GND, Drain to -ve of sensor, Sensor +ve to +3V.
 *
 * NOTE: For initial testing, wakeup time set to 5mins and battery interval to 4 cycles
 * (c) Andrew D Lindsay 2014
 */

#include "LLAPSerial.h"	// include the library
#include <OneWire.h>
#include <DallasTemperature.h>

#define VERSION_STRING "V1.0"

// Number of readings before a battery reading is taken
#define BATTERY_READ_INTERVAL 10

// Polling interval values
#define FIFTEENMINS 0
#define TENMINS 1
#define FIVEMINS 2
#define TWENTYSEC 3

// Hardware pin defines
// Enable SRF
#define SRF_RADIO_ENABLE 8
// Enable the sensor, controlled by FET
#define SENSOR_ENABLE 6
// Pin that sensor PWM input is on
#define SENSOR_PIN 3
// Using Sleep Mode 1, SLEEP must be HIGH to run
// Using Sleep Mode 2, SLEEP must be LOW to run
// Using Sleep Mode 3, SLEEP must be LOW to run
#define SRF_SLEEP 4
#define WAKE_INT 2

// Data wire is plugged into port 5 on the Arduino
#define ONE_WIRE_BUS 5

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress temperatureSensor;

boolean tempSensorFound = false;
float latestTemperature = 0.0;
uint16_t lastUncompDistance = 0;

// Define pins that are not used and are to be made inputs
uint8_t inPin[] = { 9, 10, 11, 12, 13 };

int batteryCountDown = BATTERY_READ_INTERVAL;
int pollingInterval = FIFTEENMINS;

uint8_t enterCommandMode();
// Node ID, default 00, set by input pins
// Stricly speaking this should only be digits, according to the Ciseco LLAP spec.
char nodeId[2] = { '0', '0' };

// Some functions to get the configured node address and polling
void readJumpers() {
  // Set analog input pins to read digital values, set internal pullup
  // ugly - refactor in loop.
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(9,INPUT);
  pinMode(10,INPUT);
  pinMode(12,INPUT);
  pinMode(13,INPUT);
  digitalWrite(A0, HIGH);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  digitalWrite(A4, HIGH);
  digitalWrite(A5, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  
  // Read input and parse value
  int digit2 = ((PINB & 0x30) >> 4) | ((PINC & 0x03) << 2);
  digit2 ^= 0x0f;
  int digit1 = (PINC & 0x3c) >> 2;
  digit1 ^= 0x0f;
  nodeId[0] = '0' + min(digit1,9);
  nodeId[1] = '0' + min(digit2,9);
  pollingInterval = ((PINB & 0x06) >> 1) ^ 0x03;

  Serial.println(PINB,HEX);
  Serial.println(PINC,HEX);
  
  Serial.println(pollingInterval,DEC);

  Serial.println(digit1,DEC);
  Serial.println(digit2,DEC);
  // A0, A1 are first part, R, S, T, W
  // A2 - A5 are 0-9, A-F
/*
  int a = PINC & 0x3F;
  a ^= 0x3F;
  int id1 = a & 0x03;
  switch( id1 ) {
    case 0: nodeId[0] = 'R'; break;
    case 1: nodeId[0] = 'S'; break;
    case 2: nodeId[0] = 'T'; break;
    case 3: nodeId[0] = 'W'; break;
  }
  int id2 = a >> 2;
  if( id2 <10 ) 
    nodeId[1] = '0' + id2;
  else
    nodeId[1] = 'A' + (id2-10);
*/

  // Reset analog inputs to set internal pullup off
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
}

// **************************************
// Functions used in processing readings
// Sorting function (Author: Bill Gentles, Nov. 12, 2010)
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

// Mode function, returning the mode or median.
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

// Get the range in cm. Need to enable sensor first then wait briefly for it to power up
// Also request temperture sensor, if previously detected, to take a reading.
// Disable sensor after use.
uint16_t getRange() {
  int16_t pulse;  // number of pulses from sensor
  int i=0;
  // These values are for calculating a mathematical median for a number of samples as
  // suggested by Maxbotix instead of a mathematical average
  int8_t arraysize = 9; // quantity of values to find the median (sample size). Needs to be an odd number
  //declare an array to store the samples. not necessary to zero the array values here, it just makes the code clearer
  uint16_t rangevalue[] = { 
    0, 0, 0, 0, 0, 0, 0, 0, 0 };

  float temperature = 0.0;
  digitalWrite(SENSOR_ENABLE, HIGH);
  delay(50);

  while( i < arraysize ) {								    
    pulse = pulseIn(SENSOR_PIN, HIGH);  // read in time for pin to transition
    if( pulse == 0 ) return 0;
    rangevalue[i]=pulse/58;         // pulses to centimeters (use 147 for inches)
    if( rangevalue[i] < 645 && rangevalue[i] >= 15 ) i++;  // ensure no values out of range
    delay(10);                      // wait between samples
  }
  
  digitalWrite(SENSOR_ENABLE, LOW);

  isort(rangevalue,arraysize);        // sort samples
  uint16_t distance = mode(rangevalue,arraysize);  // get median 

  lastUncompDistance = distance;
  // Use temperature comprensation if temp sensor found
  if( tempSensorFound ) {
    //temperature = sensors.getTempC(temperatureSensor);
    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0);
    latestTemperature = temperature;
//  Serial.print("Uncomp Dist: ");
//  Serial.println(distance,DEC);
//  Serial.print("temperature: ");
//  Serial.println(temperature);
    float tof = distance * 0.0058;
    uint16_t newDist = tof * (( 20.05 * sqrt( temperature + 273.15))/2);
    distance = newDist;
  }
  return distance; 
}

// End of Maxbotix sensor code
// **************************************

// Battery monitoring
int readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
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

/*

 */
uint8_t setSRFSleep() 
{
  if (!enterCommandMode())	// if failed once then try again
  {
    if (!enterCommandMode()) return 1;
  }
  // Use D9 & D10 links to determine polling interval
  // Select polling interval based on jumpers
  switch( pollingInterval ) {
    case TWENTYSEC:
      if (!sendCommand("ATSD4E20")) return 2;	// 20 seconds
      break;
 //   case ONEMIN:
 //     if (!sendCommand("ATSDEA60")) return 2;	// 1 Minute
 //     break;
    case FIVEMINS:
      if (!sendCommand("ATSD493E0")) return 2;	// 5 minutes
      break;
    case TENMINS:
      if (!sendCommand("ATSD927C0")) return 2;	// 10 minutes
      break;      
    default:
      if (!sendCommand("ATSDDBBA0")) return 2;	// 15 minutes
  }
  //if (!sendCommand("ATSDDBBA0")) return 2;	// 15 minutes
  //if (!sendCommand("ATSD927C0")) return 2;	// 10 minutes
  //if (!sendCommand("ATSD493E0")) return 2;	// 5 minutes
  //if (!sendCommand("ATSD4E20")) return 2;	// 20 seconds
  //if (!sendCommand("ATSD1388")) return 2;	// 5 seconds
  //if (!sendCommand("ATSD3E8")) return 2;	// 1 seconds
  
  if (!sendCommand("ATSM3")) return 3;
  if (!sendCommand("ATDN")) return 4;
  return 5; // success
}


uint8_t enterCommandMode()
{
  delay(1200);
  Serial.print("+++");
  delay(500);
  while (Serial.available()) Serial.read();  // flush serial in - get rid of anything received before the +++ was accepted
  delay(500);
  return checkOK(500);
}

uint8_t sendCommand(char* lpszCommand)
{
  Serial.print(lpszCommand);
  Serial.write('\r');
  return checkOK(100);
}

uint8_t checkOK(int timeout)
{
  static uint32_t time = millis();
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


void setup() {
  // initialise serial:
  Serial.begin(115200);
  // Get device ID
  readJumpers();
  // Initialise the LLAPSerial library
  LLAP.init( nodeId );
  analogReference(DEFAULT);

  // Set unused digital pins to input and turn on pull-up resistor
  for(int i = 0; i< 5; i++ ) {
    pinMode(inPin[i], INPUT);
    digitalWrite(inPin[i], HIGH);
  }

  // Setup sensor pins
  pinMode(SENSOR_ENABLE, OUTPUT);
  digitalWrite(SENSOR_ENABLE, LOW);
  pinMode(SENSOR_PIN, INPUT);

  // Setup the SRF pins
  pinMode(SRF_RADIO_ENABLE, OUTPUT);    // initialize pin 8 to control the radio
  digitalWrite(SRF_RADIO_ENABLE, HIGH); // select the radio
  pinMode(SRF_SLEEP, OUTPUT);
  digitalWrite( SRF_SLEEP, LOW );

  sensors.begin();
  tempSensorFound = false;
  if (sensors.getAddress(temperatureSensor, 0)) {
    // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
    sensors.setResolution(temperatureSensor, 9);
    tempSensorFound = true;
  }

  batteryCountDown = BATTERY_READ_INTERVAL;
  // Wait for it to be initialised
  delay(200);

  // Send STARTED message if successful or ERROR if not able to set SleepMode 3.
  // set up sleep mode 3 (low = awake)
  uint8_t val;
  while ((val = setSRFSleep()) != 5)
  {
    LLAP.sendInt("ERR",val); // Diagnostic
    delay(5000);	// try again in 5 seconds
  }

  LLAP.sendMessage("STARTED");

}

// The main loop, we basically wake up the SRF, take a reading, transmit reading then go back to sleep
void loop() {

  // Short delay to allow reading to be sent then sleep
  delay(50);
  pinMode(SRF_SLEEP, INPUT);                // sleep the radio
  LLAP.sleep(WAKE_INT, RISING, false);      // sleep until woken on pin 2, no pullup (low power)
  pinMode(SRF_SLEEP, OUTPUT);               // wake the radio
  
  // Determine if we need to send a battery voltage reading or a distance reading
  if( --batteryCountDown <= 0 ) {
    int mV = readVcc();
    LLAP.sendIntWithDP("B", mV, 3 );
    batteryCountDown = BATTERY_READ_INTERVAL;
  } 
  else {

    // Distance reading
    uint16_t cm = getRange();
    // Send temperature reading
    if( tempSensorFound ) {
      int latestTemp = (int)(latestTemperature * 100);
      LLAP.sendIntWithDP( "T", latestTemp, 2);
    }
    // Uncompensated distance
    LLAP.sendInt( "D", lastUncompDistance);
    // Send reading 3 times to make sure it gets through
    for(int n = 0; n<1; n++ ) {
      if( cm > 17 ) {
        LLAP.sendInt( "U", cm );
      } 
      else if( cm == 0 ) {
        LLAP.sendMessage( "UMax" );
      } 
      else {
        LLAP.sendMessage( "UErr" );
      }
    }
  }
}
// That's all folks

