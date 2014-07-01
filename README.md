![](https://raw.githubusercontent.com/OxFloodNet/network/master/images/icon.png)

#OxFloodNet Sensor Device

Sensor code (originally started at Sushack 2013 - http://www.sushack.co.uk/)
## Requirements
Base components:
 * Ciseco RFu-328 (SRF radio + ATMega328 module) with an ultrasonic sensor
 * Schematic and parts list shown above 

## Output 
 * LLAP to transmit sensor readings
 * Sends messages in format: a<device ID><sensor ID><value><dash padding til 12 chars>
 * Example message: aR1U65------
 * Meaning: Device R1, Ultrasonic Sensor, Reading: 65cm to water surface.
 * Example message: aR1B5500----
 * Meaning: Device R1, Battery sensor, Reading: 5500mV (5.5v remaining in battery)
 
First sensor deployed
![](https://raw.githubusercontent.com/OxFloodNet/network/master/images/First-Sensor-Deployment.jpg)

