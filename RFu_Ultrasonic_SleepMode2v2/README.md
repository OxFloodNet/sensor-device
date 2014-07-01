RFu_Ultrasonic_SleepMode2v2 - Deprecated
========================================

This code was deployed on the initial prototype sensor in Oxford in Feb 2014 and has now been replaced by https://github.com/OxFloodNet/sensor-device/tree/master/OxFloodNet_Sensor  

This version uses the RFu's sleep mode 2 to enable deep sleep low-power mode on the sensor device.  A small change is required to the Ciseco LLAP library to increase the sleep time to a longword rather than a word. The new version does not require this as it uses SRF's sleep mode instead.

The PDF schematic included in this repository was used for the hardware. There is a glaring error in this revision which leaches power constantly on pin A1 reading the battery level. This should be enabled and disabled using the NPN transistor instead. 

This schematic has been superceeded by the custom PCB.
