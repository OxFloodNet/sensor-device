Oxford Flood Network Sensor Code
=================================

Version 4.1.0 - Codename: 

Features: 
* Compatible with Maxbotix ultrasonic sensor range using PWM interface
* Dallas DS18B20 Temperature sensor for speed of sound compensation and Unique ID
* Unique 8 hex character Node number
* Variable wake timer with jumpers
* Uses an Extended LLAP message format
* Variable polling during startup phase to test device is working
* STARTvvv message sent 5 times at start, vvv indicates firmware version
* Battery message sent every 10 readings
* Reset reedswitch.
* Test mode for verifying the hardware works correctly and for remote firmware loads.

Issues: 
* None identified yet

The Oxford Flood Network sensor V4.0 is based on the Arduino core and is written for the Ciseco RFu328 which uses the ATMega328 microcontroller. The code uses some direct register acesses, so may not be directly compatible with other non-ATMega328 based boards without modification. It uses the Ciseco/Wireless Things RFu328 which includes the Ciseco/Wireless Things SRF radio transceiver module. This is used to manage the very low-power sleep mode, with the SRF waking the 328 using an interrupt pin after the wakeup time is reached.

The code is intended to be generic, with the sleep interval set by jumpers on the PCB. These are directly connected to ATMega328 pins which are read at setup() and converted into parameters for "Wakeup Period". The board has no FTDI connector pins which means that the RFu328 must either be programmed using the Ciseco RFu dev board, Voyager adaptor or programmed over the air while in test mode.

Jumpers
-------

The 2 jumpers labeled 1 and 2 represent *Wakeup Period* and has four possible options:

<pre>
 1   2    Sleep Interval
[ ] [ ] - 15 Minutes (default)
[X] [X] - 10 Minutes
[ ] [X] - 5 Minutes
[X] [ ] - 1 Minute
</pre>

The Test jumper should only be used for testing the sensor. This will allow two way communication between a laptop and sensor to perform validation on the harware. Please see the Test Mode description below for full details.

ATMega328 I/O Pin usage
-----------------------

|I/O|Use|
|---|---|
|D0|Tx SRF Radio Module|
|D1|Rx SRF Radio Module|
|D2|Wakeup interrupt|
|D3|Ultrasonic Sensor input|
|D4|SRF Sleep selection|
|D5|Temperature sensor, One Wire bus|
|D6|Ultrasonic Sensor Enable - Low to enable|
|D7|Not used|
|D8|SRF Radio enable - High to enable|
|D9|Wakeup period|
|D10|Wakeup period|
|D11|Not Used|
|D12|Not Used|
|D13|Test Mode jumper. Low selects test mode on reset|
|A0-A5|Not used|

When programming sketches using the Arduino IDE, select board type "Arduino Uno".

Test Mode
=========

The test mode is available over the wireless connection so this must be working. The RFu-328 must also have the V4 sketch loaded. A Ciseco SRFStick is required to communicate with the sensor and needs to be configured to communicate with the sensor on an alternative PANID (Network ID). An alternative PANID is used so that communcations with the sensor under test do not interfere with other working sensors and similar equipment that is located nearby.

Setting up the SRFStick
-----------------------

You need to communicate with the SRFStick using suitable Serial terminal emulation software, for example TeraTerm on Windows. To setup the correc parameters you need to enter a number of AT commands. Enter AT mode by typing `+++`, you should then receive the OK response, if not repeat again. Once you see the OK response, enter the following AT commands, ensure each gives an OK response. They set the PANID to 2305, set the packet size to maximum, make the configuration active and exit AT mode.

<pre>
+++
OK
ATID2305
OK
ATPKF0
OK
ATAC
OK
ATDN
OK
</pre>

Once setup, placing a jumperlink on the `Test` pins and resetting the sensor by sliding a magnet along the edge near the battery should result in the menu being shown.

<pre>
Oxford Flood Network Sensor
===========================

Test Menu
=========
0 - Display Menu
1 - Display Address
2 - Read Temperature
3 - Read Raw Distance
4 - Read Distance
5 - Battery Voltage
6 - Read Polling Rate
Choice: [0 - 6]
</pre>

To select an option press the number 0-6. Options 1, 5 and 6 return immediately, options 2, 3 and 4 repeatedly update the results until a key is pressed. The interval between readings is 1/2 second. This allows changes in temperature and distance to be seen. Option 0 re-displays the menu.

### Sensor Address
This displays part of the unique ID from the temperature sensor. The family ID, CRC and extra 0's are removed to give an 8 hex character node address. A value of 00000000 indicates the address was not read from the sensor and needs ot be checked.

<pre>
Address: 04BB8405
</pre>

### Read Temperature
Displays the current temperature reading. Touching the sensor should increase the reading so show that it is working. A value of 85.00C indicates that the sensor is not functioning and may need to be checked or replaced.
<pre>
Read Temperature
Temperature: 17.50C
</pre>

### Read Raw Distance
Displays the raw distance or *Error* if the sensor is outside of its usable range.
<pre>
Read Raw Distance
Raw Distance: 160cm
</pre>

### Read Distance
Displays the temperature, raw distance and compensated distance or *Error* if the sensor is outside of its usable range.
<pre>
Read Distance
Temperature: 17.50C  Raw Distance: 160cm  Distance: 159cm
</pre>

### Battery Voltage
Display the battery voltage reading.
<pre>
Read Battery Voltage
Battery: 3.3943V
</pre>

### Read Polling rate
Displays the state of the two polling/sleep interval jumpers and the rate it represents.
<pre>
Read Polling Rate
Pin 1 No Jumper
Pin 2 No Jumper
Polling: 0
15 Minutes
</pre>

