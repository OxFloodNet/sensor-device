// 
// 
// 

#include "LLAPExtended.h"

/*
const CTable __code Commands[] = {
  {9,{'A','C','K','-','-','-','-','-','-'},cmdAck},  // must be the first entry
  {0,{'A','P','V','E','R','-','-','-','-'},cmdPVer},  // Protocol version
  {0,{'D','E','V','T','Y','P','E','-','-'},cmdDevType},  // Device Type
  {0,{'D','E','V','N','A','M','E','-','-'},cmdDevName},  // Device Name
  {0,{'H','E','L','L','O','-','-','-','-'},cmdHello},  // Echo
  {3,{'S','E','R','#','#','#','#','#','#'},cmdSer},  // Serial Number
  {0,{'$','E','R','-','-','-','-','-','-'},cmdSerReset},  // Serial Number
  {0,{'F','V','E','R','-','-','-','-','-'},cmdFVer},  // Software revision
  {7,{'C','H','D','E','V','I','D','#','#'},cmdDevID},  // Device ID
  {5,{'P','A','N','I','D','#','#','#','#'},cmdPanID},  // PANID
  {0,{'R','E','B','O','O','T','-','-','-'},cmdReset},  // reset
  {7,{'R','E','T','R','I','E','S','#','#'},cmdRetries},  // set # retrys
  {4,{'B','A','T','T','-','-','-','-','-'},cmdBatt}, // request battery voltage
  {4,{'S','A','V','E','-','-','-','-','-'},cmdSave}, // Save config to flash
#if defined(APSLEEP)  // cyclic sleep
  {0,{'I','N','T','V','L','#','#','#','#'},cmdInterval},  // SET cyclic sleep interval - 999S - three digits + timescale
                                                          // T=mS, S=S, M=mins, H=Hours, D=days
  {0,{'C','Y','C','L','E','-','-','-','-'},cmdCyclic},  // activate cyclic sleep
  {0,{'W','A','K','E','-','-','-','-','-'},cmdDeactivate},  // deactivate programmed behaviour (cyclic sleep etc)
  {5,{'W','A','K','E','C','#','#','#','-'},cmdSetSleepCount}, // set the sleep count until awake is sent
#endif
// allow all device to do a one shot sleep (including DALLAS)
  {0,{'S','L','E','E','P','#','#','#','#'},cmdActivate},  // activate Sleeping mode - one shot or sleep until interrupt
*/

void LLAPExtended::init()
{
	sMessage.reserve(10);
	bMsgReceived = false;
	for( byte i=0; i<8; i++ )
		deviceId[i] = '-';
}

void LLAPExtended::init(char* dID)
{
	init();
	bMsgReceived = false;
	setDeviceId(dID);
	cMessage[18]=0;		// ensure terminated
}

void LLAPExtended::processMessage(){
	//if (LLAP.cMessage[0] != 'a') return; //not needed as already checked
//	if (cMessage[1] != deviceId[0]) return;
//	if (cMessage[2] != deviceId[1]) return;
	// now we have LLAP.cMessage[3] to LLAP.cMessage[11] as the actual message
	if (0 == strncmp_P(&cMessage[3],PSTR("HELLO----"),9)) {
		Serial.print(cMessage);	// echo the message
//	} else if (0 == strncmp_P(&cMessage[3],PSTR("CHDEVID"),7)) {
//	  if (strchr_P(PSTR("-#@?\\*ABCDEFGHIJKLMNOPQRSTUVWXYZ"), cMessage[10]) != 0 && strchr_P(PSTR("-#@?\\*ABCDEFGHIJKLMNOPQRSTUVWXYZ"), cMessage[11]) != 0)
//	  {
//		deviceId[0] = cMessage[10];
//		deviceId[1] = cMessage[11];
//		Serial.print(cMessage);	// echo the message
//	  }
	} else {
		sMessage = String(&cMessage[3]); // let the main program deal with it
		bMsgReceived = true;
	}
}

void LLAPExtended::SerialEvent()
{
	if (bMsgReceived) return; //get out if previous message not yet processed
	if (Serial.available() >= 18) {
        // get the new byte:
        char inChar = (char)Serial.peek();
        if (inChar == 'b') {
            for (byte i = 0; i<18; i++) {
                inChar = (char)Serial.read();
                cMessage[i] = inChar;
                if (i < 11 && Serial.peek() == 'a') {
                    // out of synch so abort and pick it up next time round
                    return;
                }
            }
            cMessage[18]=0;
            processMessage();
        }
        else
            Serial.read();	// throw away the character
    }
}

void LLAPExtended::sendMessage(String sToSend)
{
    cMessage[0] = 'b';
    for( byte i=0; i<8; i++ )
	cMessage[i+1] = deviceId[i];
    for (byte i = 0; i<9; i++) {
		if (i < sToSend.length())
			cMessage[i+9] = sToSend.charAt(i);
		else
			cMessage[i+9] = '-';
    }
    
    Serial.print(cMessage);
    Serial.flush();
}

void LLAPExtended::sendMessage(char* sToSend)
{
	sendMessage(sToSend,NULL);
}

void LLAPExtended::sendMessage(char* sToSend, char* valueToSend)
{
    cMessage[0] = 'b';
    for( byte i=0; i<8; i++ )
	cMessage[i+1] = deviceId[i];
    for (byte i = 0; i<9; i++) {
		if (i < strlen(sToSend))
			cMessage[i+9] = sToSend[i];
		else if (i < strlen(sToSend) + strlen(valueToSend))
			cMessage[i+9] = valueToSend[i - strlen(sToSend)];
		else
			cMessage[i+9] = '-';
    }
    
    Serial.print(cMessage);
    Serial.flush();
}

void LLAPExtended::sendMessage(const __FlashStringHelper *ifsh)
{
	sendMessage(ifsh,NULL);
}

void LLAPExtended::sendMessage(const __FlashStringHelper *ifsh, char* valueToSend)
{
    const char PROGMEM *p = (const char PROGMEM *)ifsh;
    byte eos = 0;
    cMessage[0] = 'b';
    for( byte i=0; i<8; i++ )
	cMessage[i+1] = deviceId[i];
    for (byte i = 0; i<9; i++) {
		if (!eos)
		{
			cMessage[i+9] = pgm_read_byte(p++);
			if (!cMessage[i+9]) // end of string
			{
				eos = i-9;
			}
		}
		if (eos)
		{
			if (i < eos + strlen(valueToSend))
				cMessage[i+9] = valueToSend[i - eos];
			else
				cMessage[i+9] = '-';
		}
    }
    Serial.print(cMessage);
    Serial.flush();
}

void LLAPExtended::sendInt(String sToSend, int value)
{
	char cValue[7];		// long enough for -32767 and the trailing zero
	itoa(value, cValue,10);
	byte cValuePtr = 0;

    cMessage[0] = 'b';
    for( byte i=0; i<8; i++ )
	cMessage[i+1] = deviceId[i];
    for (byte i = 0; i<9; i++) {
		if (i < sToSend.length())
			cMessage[i+9] = sToSend.charAt(i);
		else if (cValuePtr < 7 && cValue[cValuePtr] !=0)
			cMessage[i+9] = cValue[cValuePtr++];
		else
			cMessage[i+9] = '-';
    }
    
    Serial.print(cMessage);
    Serial.flush();
}

void LLAPExtended::sendIntWithDP(String sToSend, int value, byte decimalPlaces)
{
	char cValue[8];		// long enough for -3276.7 and the trailing zero
	byte cValuePtr=0;
	itoa(value, cValue,10);
	char* cp = &cValue[strlen(cValue)];
	*(cp+1) = 0;	// new terminator
	while (decimalPlaces-- && --cp )
	{
		*(cp+1) = *cp;
	}
	*cp = '.';

    cMessage[0] = 'b';
    for( byte i=0; i<8; i++ )
	cMessage[i+1] = deviceId[i];
    for (byte i = 0; i<9; i++) {
		if (i < sToSend.length())
			cMessage[i+9] = sToSend.charAt(i);
		else if (cValuePtr < 8 && cValue[cValuePtr] !=0)
			cMessage[i+9] = cValue[cValuePtr++];
		else
			cMessage[i+9] = '-';
    }
    
    Serial.print(cMessage);
    Serial.flush();
}

void LLAPExtended::setDeviceId(char* cId)
{
    for(byte i=0; i< 8; i++ )
    	deviceId[i] = cId[i];
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This power-saving code was shamelessly stolen from the Jeelabs library with slight modification.
// see https://github.com/jcw/jeelib
// The watchdog timer is only about 10% accurate - varies between chips


#include <avr/sleep.h>
#include <util/atomic.h>

static volatile byte watchdogCounter;

void watchdogEvent() {
    ++watchdogCounter;
}

ISR(WDT_vect) { watchdogEvent(); }


void watchdogInterrupts (char mode) {
    // correct for the fact that WDP3 is *not* in bit position 3!
    if (mode & bit(3))
        mode ^= bit(3) | bit(WDP3);
    // pre-calculate the WDTCSR value, can't do it inside the timed sequence
    // we only generate interrupts, no reset
    byte wdtcsr = mode >= 0 ? bit(WDIE) | mode : 0;
    MCUSR &= ~(1<<WDRF);
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
#ifndef WDTCSR
#define WDTCSR WDTCR
#endif
        WDTCSR |= (1<<WDCE) | (1<<WDE); // timed sequence
        WDTCSR = wdtcsr;
    }
}

/// @see http://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html
void powerDown () {
    byte adcsraSave = ADCSRA;
    ADCSRA &= ~ bit(ADEN); // disable the ADC
    // switch off analog comparator - not in Jeelabs' code
    ACSR = ACSR & 0x7F; // note if using it then we need to switch this back on when we wake.
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        sleep_enable();
        // sleep_bod_disable(); // can't use this - not in my avr-libc version!
#ifdef BODSE
        MCUCR = MCUCR | bit(BODSE) | bit(BODS); // timed sequence
        MCUCR = (MCUCR & ~ bit(BODSE)) | bit(BODS);
#endif
    }
    sleep_cpu();
    sleep_disable();
    // re-enable what we disabled
    ADCSRA = adcsraSave;
}

byte LLAPExtended::sleepForaWhile (long msecs) {
    byte ok = 1;
    long msleft = msecs;
    // only slow down for periods longer than the watchdog granularity
    while (msleft >= 16) {
        char wdp = 0; // wdp 0..9 corresponds to roughly 16..8192 ms
        // calc wdp as log2(msleft/16), i.e. loop & inc while next value is ok
        for (word m = msleft; m >= 32; m >>= 1)
            if (++wdp >= 9)
                break;
        watchdogCounter = 0;
        watchdogInterrupts(wdp);
        powerDown();
        watchdogInterrupts(-1); // off
        // when interrupted, our best guess is that half the time has passed
        long halfms = 8 << wdp;
        msleft -= halfms;
        if (watchdogCounter == 0) {
            ok = 0; // lost some time, but got interrupted
            break;
        }
        msleft -= halfms;
    }
    // adjust the milli ticks, since we will have missed several
#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny85__) || defined (__AVR_ATtiny44__) || defined (__AVR_ATtiny45__)
    extern volatile unsigned long millis_timer_millis;
    millis_timer_millis += msecs - msleft;
#else
    extern volatile unsigned long timer0_millis;
    timer0_millis += msecs - msleft;
#endif
    return ok; // true if we lost approx the time planned
}

void pin2_isr()
{
  sleep_disable();
  detachInterrupt(0);
}

void pin3_isr()
{
  sleep_disable();
  detachInterrupt(1);
}

void LLAPExtended::sleep(byte pinToWakeOn, byte direction, byte bPullup)	// full sleep wake on interrupt - pin is 2 or 3
{
  byte adcsraSave = ADCSRA;
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  // switch off analog comparator - not in Jeelabs' code
  ACSR = ACSR & 0x7F; // note if using it then we need to switch this back on when we wake.
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  if (pinToWakeOn == 2)
  {
	pinMode(2,INPUT);
    if (bPullup) digitalWrite(2,HIGH);		// enable pullup
	attachInterrupt(0, pin2_isr, direction);
  }
  else
  {
	pinMode(3,INPUT);
    if (bPullup) digitalWrite(3,HIGH);		// enable pullup
	attachInterrupt(1, pin3_isr, direction);
  }
  cli();
  // sleep_bod_disable(); // can't use this - not in my avr-libc version!
#ifdef BODSE
    MCUCR = MCUCR | bit(BODSE) | bit(BODS); // timed sequence
    MCUCR = (MCUCR & ~ bit(BODSE)) | bit(BODS);
#endif
  sei();
  sleep_cpu();	// and wait until we are woken
  sleep_disable();
  // re-enable what we disabled
  ADCSRA = adcsraSave;
}

// End of power-saving code.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
	LLAP.SerialEvent();
}


LLAPExtended LLAP;	// declare the instance

