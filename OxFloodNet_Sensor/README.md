Oxford Flood Network Sensor Code
=================================

Version 3.1.4 - Codename: Bulstake

Features: 
* Variable Node ID with jumpers
* Variable wake timer with jumpers
* Variable poling during startup phase to test device is working
* Dallas DS18B20 Temperature sensor for speed of sound compensation
* Compatible with Maxbotix ultrasonic sensor range using PWM interface
* STARTvvv message sent 5 times at start, vvv indicates firmware version
* Battery message sent every 10 readings

Issues: 
* >Max reading leads to no LLAP message being sent instead of error
* After programming unplug FTDI cable before trying normal operation (TX/RX get diverted)

OxFloodNet sensor V3.1 uses Arduino code. It's written for the Ciseco RFu328 which uses an ATMega328. The code uses some register settings, so is not directly compatible with other boards without modification. It relies on the Ciseco RFu328 which includes the Ciseco SRF radio transceiver. This is needed to manage the very low-power sleep modes, with the SRF waking the 328 up using an interrupt pin after a sleep timer completes.

The code is intended to be generic, with parameters being set by jumpers on the PCB. These are directly connected to ATMega328 pins which are read at setup() and converted into parameters for "Node ID" and "Wakeup Period". The board has no FTDI chip which means an FTDI programming cable is needed to update the code (must be removed again for board to operate properly)

![OxFloodNet v3.1 PCB Jumper Layout](https://raw.githubusercontent.com/OxFloodNet/sensor-device/master/OxFloodNet_Sensor/2014-09-11%2020.52.24.jpg "Jumper Layout")

In the image above, the jumpers should be read from left to right: 
<pre>
A5 ,A4 ,A3 ,A2   - Node ID high digit
A1 ,A0 ,D13,D12  - Node ID low digit
D10, D9          - Wakeup Period (backwards?)


The first eight represent the *Node ID* as two nibbles, four jumpers each:
<pre>

 High Hex Digit:
  8   4   2   1  (binary digit)
 A5  A4  A3  A2  (ATMega328 pin)
 
 Low Hex Digit: 
  8   4   2   1  (binary digit)
 A1  A0 D13 D12  (ATMega328 pin)
 
 Examples:
  Node ID 0x21 would be a jumper on A3 (2) and D12.   
  Jumper A4&A3&A0 would make 0x64. 
  Jumper D13&A5&A4 would make high digit: (8+4 = B) and low digit: (2) = 0xB2

</pre>

The 9th & 10th jumpers from the left represent *Wakeup Period* and has four possible values mapped
to a number of minutes to sleep for:
<pre>
D9  D10 Value
[ ] [ ] 0      - 15 Minutes (default)
[ ] [x] 1      - 10 Minutes
[x] [ ] 2      - 5 Minutes
[x] [x] 3      - 1 Minute
</pre>

The last jumper on the right is unused but is marked PWR. This for possible expanstion and additional sensors.


<pre>
Pin usage on ATMega328:

D3 - Sensor input
D5 - Temperature sensor, One Wire bus
D6 - Sensor Enable - Low to enable
D8 - SRF Radio enable - High to enable
D9 - Wakeup period 0
D10 - Wakeup period 1
D12 - First Node ID digit bit 0
D13 - First Node ID digit bit 1
A0 - First Node ID digit bit 2
A1 - First Node ID digit bit 3
A2 - Second Node ID digit bit 0
A3 - Second Node ID digit bit 1
A4 - Second Node ID digit bit 2
A5 - Second Node ID digit bit 3

AVR mapping
A0-5 - PC0 - PC5
D12, D13 - PB4, PB5
D9,D10 - PB1, PB2

Board type in Arduino IDE should be set to "Arduino Uno"

</pre>
