Oxford flood network sensor 

OxFloodNet sensor V3.1 uses Arduino code. It's written for the Ciseco RFu328 which uses an ATMega328. The code uses some register settings, so is not directly compatible with other boards without modification. It relies on the Ciseco RFu328 which includes the Ciseco SRF radio transceiver. This is needed to manage the very low-power sleep modes, with the SRF waking the 328 up using an interrupt pin after a sleep timer completes.

The code is intended to be generic, with parameters being set by jumpers on the PCB. These are directly connected to ATMega328 pins which are read at setup() and converted into parameters for "Node ID" and "Wakeup Period".


Jumpers & Pins
<pre>
Set Wakeup Period
D9  D10 Value
[ ] [ ] 0      - 15 Minutes (default)
[ ] [x] 1      - 10 Minutes
[x] [ ] 2      - 5 Minutes
[x] [x] 3      - 20 Seconds

Set Node ID (consists of 2 x 4-pin bytes):
 First Digit:
  8   4   2   1  (binary digit)
 A5  A4  A3  A2  (ATMega328 pin)
 
 Second Digit: 
  8   4   2   1  (binary digit)
 A1  A0 D13 D12  (ATMega328 pin)
 
 So, for example, node ID 21 would be a jumper on A3 and D12.   Jumper A4&A3&A0 would make 64.


Pin usage on ATMega328:

D3 - Sensor input
D5 - Temperature sensor, One Wire bus
D6 - Sensor Enable - Low to enable
D8 - SRF Radio enable - ??? to enable
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
