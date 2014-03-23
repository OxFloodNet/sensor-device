//////////////////////////////////////////////////////////////////////////
// Simple RFu SRF LLAP Tx Test example
//
// Works well on RFu328. Issue with Xino RF where the sleep mode disable the USB serial port opperation! 
// 
//
// Uses the Ciseco LLAPSerial library
//
// Example by Glyn Hudson OpenEnergyMonitor.org June 2013
// Based on LLAP Serial Library and examples from Ciseco
//////////////////////////////////////////////////////////////////////////

#include <LLAPSerial.h>

#define DEVICEID "A1"	// this is the LLAP device ID

int i;
float f;


void setup() {
       	Serial.begin(115200);

	pinMode(8,OUTPUT);		// switch on the SRF radio
	digitalWrite(8,HIGH);
	delay(1000);		        // allow the radio to startup

//-------------enable SRF sleep mode 2---------------------------------------------- 
//http://openmicros.org/index.php/articles/88-ciseco-product-documentation/260-srf-configuration       
        pinMode(4,OUTPUT);           // hardwired XinoRF / RFu328 SRF sleep pin 
        digitalWrite(4,LOW);          // pull sleep pin high - sleep 2 disabled
	Serial.print("+++");            // enter AT command mode
        delay(1500);                   // delay 1.5s
        Serial.println("ATSM2");         // enable sleep mode 2 <0.5uA
        delay(2000);
        Serial.println("ATDN");          // exit AT command mode*/
        delay(2000);
//---------------------------------------------------------------------------------

        LLAP.init(DEVICEID);
	LLAP.sendMessage("STARTED");
	//tst code
	Serial.print("ABCDEFGHIJKLMNOPQRSTUVWX");
	Serial.flush();

       
}   

void loop() {
  
        //LLAP receive code - not used in this low power Tx only example 
        // print the string when a newline arrives:
	//if (LLAP.bMsgReceived) {
	//	Serial.print("message is:");
	//	Serial.println(LLAP.sMessage); 
	//	LLAP.bMsgReceived = false;	// if we do not clear the message flag then message processing will be blocked
	//}

//--------Send Test Variables----------------------------------------------------------
  LLAP.sendMessage("TEST0");
  LLAP.sendInt("TEST1",i);              //name, int variable
  LLAP.sendIntWithDP("Test2", f ,1);    //name, float variable, number of DP's
  i++;                          	//inc test variables        
  f=f+0.1;
//-------------------------------------------------------------------------------------
		
//-------Put ATmega328 and SRF Radio to sleep------------------------------------------
  delay(10);                    // allow radio to finish sending
  //Serial.println();
  digitalWrite(4, HIGH);        // pull sleep pin high to enter SRF sleep 2
  LLAP.sleepForaWhile(5000);    // sleep ATmega328 for 5s (ms)
  digitalWrite(4, LOW);         // when ATmega328 wakes up, wake up SRF Radio
  delay(10);                    // allow radio to wake up
//-------------------------------------------------------------------------------------
	
}
