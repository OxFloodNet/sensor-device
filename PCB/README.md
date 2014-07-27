This repository contains the EAGLE files and gerbers for creating an Oxfloodnet Sensor PCB v3.1. 

The sensor device has the following features:
* Arduino-compatible code wireless sensor node
* Wireless
  * Transmits using 433, 868 or 915* MHz  (*USA only)
  * LLAP Protocol
  * Up to 200m range
  * Chip, Wire Whip and External (u.FL) antenna options
* Power: 
  * Option for Long-life (1 x 3.6v Tadiran Lithium Cell) or Off-the Shelf (2 x 1.6v AA Batteries)
  * Battery voltage reading sent every 10th cycle
  * Extremely low power sleep mode (~30uA)
* Maxbotix ultrasonic ranger
  * Compatible with most Maxbotix sensors
  * Maxbotix MB7060 XL-MaxSonar-WR1 IP67 outdoor-rated sensor recommended due to humidity/condensation issues
  * Maxbotix MB1230 works but has condensation issues above streams
* DS18B20 temperature sensor 
  * to compensate for speed of sound in different conditions
  * Temperature data also a useful byproduct
  * Molex header connection to board for easy maintenance
* Jumpers/Headers
  * Control poll cycle (10s, 5, 10 or 15 minutes depending on jumpers)
  * FTDI connector for programming sketch to RFu-328 (not powered)
  * Molex temperature connector
  * Maxbotix 7-way header
  
See Arduino [device code](https://github.com/OxFloodNet/sensor-device/tree/master/OxFloodNet_Sensor) for more specific info about controlling these.

A schematic of the Oxfloodnet Sensor PCB v3.1  
![](https://raw.githubusercontent.com/OxFloodNet/sensor-device/master/PCB/oxfloodnet_sensorv3.png)

The assembled v3.0 sensor can be seen in the pictures below along with aseembly instructions at [http://blog.thiseldo.co.uk/?p=1285](http://blog.thiseldo.co.uk/?p=1285). V3.1 assembly is similar, instructions will follow later.

![](https://raw.githubusercontent.com/OxFloodNet/sensor-device/master/PCB/2014-06-16%2019.46.09.jpg)
![](https://raw.githubusercontent.com/OxFloodNet/sensor-device/master/PCB/2014-06-16%2020.43.26.jpg)
![](https://raw.githubusercontent.com/OxFloodNet/sensor-device/master/PCB/2014-06-16%2021.30.13.jpg)
![](https://raw.githubusercontent.com/OxFloodNet/sensor-device/master/PCB/2014-06-18%2021.20.05.jpg)

