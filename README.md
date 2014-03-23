#OxFloodNet Sensor Device
Sensor code (originally started at Sushack 2013 - http://www.sushack.co.uk/)

## Requirements
Base components:
 * Ciseco RFu-328 (SRF radio + ATMega328 module) with an ultrasonic sensor
 * Schematic and parts list shown above 

![](https://raw.github.com/sushack/sensor-firmware/master/2013-11-003.jpg)
  
## Output 
 * LLAP to transmit sensor readings
 * Sends messages in format: a<device ID><sensor ID><value><dash padding til 12 chars>
 * Example message: aR1U65------
 * Meaning: Device R1, Ultrasonic Sensor, Reading: 65cm to water surface.

Sensor gateway:
 * Raspberry Pi running a Ciseco SRF radio and LLAP listener
 * Code at: https://github.com/sushack/pi_sensor_mqtt
 * TODO: Alternative code for Node-Red
 
First sensor deployed
![](https://raw.github.com/sushack/sensor-firmware/master/First-Sensor-Deployment.jpg)
