
#include "LLAPSerial.h"	// include the library

void setup() {
  // initialise serial:
  Serial.begin(9600);
  // Initialise the LLAPSerial library
  LLAP.init();
}

void loop() {
	// print the string when a newline arrives:
	if (LLAP.bMsgReceived) {
		Serial.print("message is:");
		Serial.println(LLAP.sMessage); 
		LLAP.bMsgReceived = false;	// if we do not clear the message flag then message processing will be blocked
	}
}




