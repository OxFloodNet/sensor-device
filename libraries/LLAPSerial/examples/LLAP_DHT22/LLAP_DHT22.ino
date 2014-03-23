//////////////////////////////////////////////////////////////////////////
// LLAP temperature and humidity sensor using a DHT22
//
// Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
//
// will work with any Arduino compatible however the target boards are the 
// Ciseco XinoRF and RFu-328, for LLAP over radio
// 
//
// Uses the Ciseco LLAPSerial library
// Uses the Adafruit DHT library https://github.com/adafruit/DHT-sensor-library
//////////////////////////////////////////////////////////////////////////

#include <LLAPSerial.h>
#include <DHT.h>

#define DEVICEID "DH"	// this is the LLAP device ID

#define DHTPIN 2     // what I/O the DHT-22 data pin is connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);

void setup() {
	Serial.begin(115200);
	pinMode(8,OUTPUT);		// switch on the radio
	digitalWrite(8,HIGH);
	pinMode(4,OUTPUT);		// switch on the radio
	digitalWrite(4,LOW);	// ensure the radio is not sleeping
	delay(1000);				// allow the radio to startup
	LLAP.init(DEVICEID);

	dht.begin();

	LLAP.sendMessage(F("STARTED"));

}

void loop() {
	// print the string when a newline arrives:
	if (LLAP.bMsgReceived) {
		Serial.print(F("msg:"));
		Serial.println(LLAP.sMessage); 
		LLAP.bMsgReceived = false;	// if we do not clear the message flag then message processing will be blocked
	}

	// every 30 seconds
	static unsigned long lastTime = millis();
	if (millis() - lastTime >= 30000)
	{
  		lastTime = millis();
		int h = dht.readHumidity() * 10;
		int t = dht.readTemperature() * 10;
 		// check if returns are valid, if they are NaN (not a number) then something went wrong!
		if (isnan(t) || isnan(h)) {
			LLAP.sendMessage(F("ERROR"));
		} else {
			LLAP.sendIntWithDP("HUM",h,1);
			//delay(100);
			LLAP.sendIntWithDP("TMP",t,1);
		}
	}
}
