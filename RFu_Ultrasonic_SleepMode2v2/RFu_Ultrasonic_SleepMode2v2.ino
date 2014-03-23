/* OxFloodNet Sensor Code */
/* Developed by Ben Ward, Andrew Lindsay, Nick Kolpin

Board: Ciseco RFu - ATMega 328  + SRF Radio
Sensor: Seeed studios Ultrasonic rangefinder
*/

#include <avr/power.h>
#include <Ultrasonic.h>
#include <LLAPSerial.h>

#define DEVICEID "R5"	    // this is the LLAP device ID
#define RADIO_ENABLE 8      // this pin enables the radio!
#define RADIO_SLEEP 4       // this controls the radio's sleep 
#define ULTRASONIC_PIN 5    // this is the ultrasonic SIG pin
#define ULTRASONIC_ENABLE 9 // this is the power to the ultrasonic sensor
#define SLEEP_TIME 300000L  // Amount of time to sleep. Longword to hold larger number, but requires modification to LLAP library

uint8_t inPin[] = {  2, 3, 7, 10, 11, 12, 13 };  // List of pins to turn off later

float vbatt;   // for battery voltage
int adc;       // for voltage divider

Ultrasonic ultrasonic(ULTRASONIC_PIN); // Create ultrasonic object

void setup() {
     
  byte intCounter, adcsra, mcucr1, mcucr2;

  // Disable as much as we can to reduce power usage
//  ADCSRA &= ~(1<<ADEN); //Disable ADC
//  ACSR = (1<<ACD); //Disable the analog comparator
  DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins
  DIDR1 = (1<<AIN1D)|(1<<AIN0D); //Disable digital input buffer on AIN1/0
  cli();
  mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  //turn off the brown-out detector
  mcucr2 = mcucr1 & ~_BV(BODSE);
  MCUCR = mcucr1;
  MCUCR = mcucr2;
  sei();   
//  power_adc_disable();
  power_spi_disable();
  power_twi_disable();
  power_timer1_disable();
 
   // Set unused digital pins to input and turn on pull-up resistor
  for(int i = 0; i< 8; i++ ) {
    pinMode(inPin[i], INPUT);
    digitalWrite(inPin[i], HIGH);
  }
 
  
  	Serial.begin(115200);              // Start the serial port (also attached to the Radio). ATMega 328 only runs at 115200
     
        pinMode(ULTRASONIC_PIN, OUTPUT);      // Set ultrasonic polling pin
        pinMode(ULTRASONIC_ENABLE, OUTPUT);   // Set ultrasonic turn on and off pin
        digitalWrite(ULTRASONIC_ENABLE, LOW); // Turn ultrasonic off
      
        pinMode(RADIO_ENABLE,OUTPUT);		// Set pin to switch the radio on
	digitalWrite(RADIO_ENABLE,HIGH);           // switch on the SRF radio
	delay(1000);		        // allow the radio to startup

        // enable SRF sleep mode 2 by entering AT commands
        // send AT commands via serial port which the radio also listens to 
        // See: http://openmicros.org/index.php/articles/88-ciseco-product-documentation/260-srf-configuration       
 
        pinMode(RADIO_SLEEP,OUTPUT);             // hardwired RFu328 SRF sleep pin 
        digitalWrite(RADIO_SLEEP,LOW);           // pull sleep pin low (awake) ready for when we set Sleep mode 2 (don't fall asleep halfway through)

	Serial.print("+++");           // enter AT command mode
        delay(1500);                   // delay 1.5s
        Serial.println("ATSM2");       // enable sleep mode 2 <0.5uA
        delay(2000);                    
        Serial.println("ATDN");        // exit AT command mode*/
        delay(2000);

        /* Start up radio comms */
        LLAP.init(DEVICEID);            // announce our ID
	LLAP.sendMessage("STARTED");    // announce our start

      
}   


void loop() {
  
  /* Read the untrasonic sensor distance */  
  digitalWrite(ULTRASONIC_ENABLE, HIGH);          // turn on the ultrasonic sensor
  delay(100);                                     // wait for it
  long cm = ultrasonic.MeasureInCentimeters();    // measure the range
  delay(1000);                                    // wait for it to read
  digitalWrite(ULTRASONIC_ENABLE, LOW);           // turn off the ultrasonic

  /* Transmit the Ultrasonic sensor reading */
  LLAP.sendInt( "U", cm );      // Send a reading labelled "Ultrasound"
  delay(100);                    // allow radio to finish sending


  /* Calculate battery level */
  int adc = analogRead(A1);                  // read the voltage divider
  float vbatt = adc * (3.3 / 1023.0) * 2;    // calculate voltage from reference 
  
  /* Transmit battery reading in mV */
  LLAP.sendInt("B",vbatt*1000);              // Send a reading labelled "Battery"
  delay(100);                                // pause 

	
  /* Put the radio and ATMega 328 to sleep */
  delay(100);                               // allow radio to finish sending
  digitalWrite(RADIO_SLEEP, HIGH);          // pull sleep pin high to enter SRF sleep mode 2
    // NOTE: If SLEEP_TIME is greater than a word length, 32767 then update
  // function to use a long instead of word.
  LLAP.sleepForaWhile ( SLEEP_TIME );
  
  digitalWrite(RADIO_SLEEP, LOW);           // when ATmega328 wakes up, wake up SRF Radio
  delay(100);                               // allow radio to wake up
  
  // repeat
}



