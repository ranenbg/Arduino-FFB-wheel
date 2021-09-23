/*
  wiring.h - Partial implementation of the Wiring API for the ATmega8.
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id$
*/

#ifndef Wiring_h
#define Wiring_h

#include <avr/io.h>
#include <stdlib.h>
#include "binary.h"
#include "wiring_private.h"
#include "pins_arduino.h"
#include <digital_write_macros.h>


#ifdef __cplusplus
extern "C"{
#endif

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1

#define true 0x1
#define false 0x0

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define INTERNAL1V1 2
#define INTERNAL2V56 3
#else
#define INTERNAL 3
#endif
#define DEFAULT 1
#define EXTERNAL 0

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define interrupts() sei()
#define noInterrupts() cli()

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
	

	
typedef unsigned int word;

#define bit(b) (1UL << (b))

typedef uint8_t boolean;
typedef uint8_t byte;

void init(void);

int analogRead(uint8_t);
void analogReference(uint8_t mode);
void analogWrite(uint8_t, int);

void __pinMode(uint8_t, uint8_t);
void __digitalWrite(uint8_t, uint8_t);
int __digitalRead(uint8_t);
	//The next 4 are lies--macros are used
	void pinMode(uint8_t, uint8_t);
	void digitalWrite(uint8_t, uint8_t);
	int digitalRead(uint8_t);
	void noAnalogWrite(uint8_t);

/*	
inline void turnOffPWM(uint8_t timer) __attribute__ ((always_inline)); 
inline void turnOffPWM(uint8_t timer){  
*/
static inline void turnOffPWM(uint8_t timer) {
	// Forcing this inline keeps the callers from having to push their own stuff
	// on the stack. It is a good performance win and only takes 1 more byte per
	// user than calling. (It will take more bytes on the 168.)
	//
	// But shouldn't this be moved into pinMode? Seems silly to check and do on
	// each digitalread or write.
	//
		if (timer == TIMER1A) cbi(TCCR1A, COM1A1);
		if (timer == TIMER1B) cbi(TCCR1A, COM1B1);
		
#if defined(__AVR_ATmega8__)
		if (timer == TIMER2) cbi(TCCR2, COM21);
#else
		if (timer == TIMER0A) cbi(TCCR0A, COM0A1);
		if (timer == TIMER0B) cbi(TCCR0A, COM0B1);
		if (timer == TIMER2A) cbi(TCCR2A, COM2A1);
		if (timer == TIMER2B) cbi(TCCR2A, COM2B1);
#endif
		
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
		if (timer == TIMER3A) cbi(TCCR3A, COM3A1);
		if (timer == TIMER3B) cbi(TCCR3A, COM3B1);
		if (timer == TIMER3C) cbi(TCCR3A, COM3C1);
		if (timer == TIMER4A) cbi(TCCR4A, COM4A1);
		if (timer == TIMER4B) cbi(TCCR4A, COM4B1);
		if (timer == TIMER4C) cbi(TCCR4A, COM4C1);
		if (timer == TIMER5A) cbi(TCCR5A, COM5A1);
		if (timer == TIMER5B) cbi(TCCR5A, COM5B1);
		if (timer == TIMER5C) cbi(TCCR5A, COM5C1);
#endif
	}
	
#define __atomicWrite__(A,P,V) \
if ( (int)(A) < 0x40) { bitWrite(*((volatile uint8_t*)A), __digitalPinToBit(P), (V) );}  \
else {                                                         \
uint8_t register saveSreg = SREG;                          \
cli();                                                     \
bitWrite(*((volatile uint8_t*)A), __digitalPinToBit(P), (V) );                   \
SREG=saveSreg;                                             \
} 
	
#define digitalWrite(P, V) \
do {                       \
if (__builtin_constant_p(P) && __builtin_constant_p(V))   __atomicWrite__((uint8_t*) digitalPinToPortReg(P),P,V) \
else  __digitalWrite((P), (V));         \
}while (0)
	
#define pinMode(P, V) \
do {if (__builtin_constant_p(P) && __builtin_constant_p(V)) __atomicWrite__((uint8_t*) digitalPinToDDRReg(P),P,V) \
else __pinMode((P), (V)); \
} while (0)

#define digitalRead(P) ( (int) _digitalReadFast2_((P)) )
#define _digitalReadFast2_(P ) \
(__builtin_constant_p(P) ) ? ( \
( bitRead(*digitalPinToPINReg(P), __digitalPinToBit(P))) ) : \
__digitalRead((P))
	
#define noAnalogWrite(P) turnOffPWM(digitalPinToTimer((P)))
	
unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long);
void delayMicroseconds(unsigned int us);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

void attachInterrupt(uint8_t, void (*)(void), int mode);
void detachInterrupt(uint8_t);

void setup(void);
void loop(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif  Wiring_H

//===========================================================================================
/* faster digitalWrite, digitalRead, pinMode for Arduino
 * The basic scheme for this was developed by Paul Stoffregen with digitalWrite.
 * extended to pinMode by John Raines and 
 * extended to digitalRead by John Raines with considerable assistance by William Westfield
 * Copyright (c) 2008-2010 PJRC.COM, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/*
#ifndef DigitalWrite_H
#define DigitalWrite_H

#ifndef bitWrite
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#endif

#if !defined(digitalPinToPortReg)
#if !(defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) )	

// Standard Arduino Pins
#define digitalPinToPortReg(P) \
(((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define digitalPinToDDRReg(P) \
(((P) >= 0 && (P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define digitalPinToPINReg(P) \
(((P) >= 0 && (P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define __digitalPinToBit(P) \
(((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P) - 8 : (P) - 14))

#if defined(__AVR_ATmega8__)

// 3 PWM
#define __digitalPinToTimer(P) \
(((P) ==  9 || (P) == 10) ? &TCCR1A : (((P) == 11) ? &TCCR2 : 0))
#define __digitalPinToTimerBit(P) \
(((P) ==  9) ? COM1A1 : (((P) == 10) ? COM1B1 : COM21))
#else  //168,328

// 6 PWM
#define __digitalPinToTimer(P) \
(((P) ==  6 || (P) ==  5) ? &TCCR0A : \
(((P) ==  9 || (P) == 10) ? &TCCR1A : \
(((P) == 11 || (P) ==  3) ? &TCCR2A : 0)))
#define __digitalPinToTimerBit(P) \
(((P) ==  6) ? COM0A1 : (((P) ==  5) ? COM0B1 : \
(((P) ==  9) ? COM1A1 : (((P) == 10) ? COM1B1 : \
(((P) == 11) ? COM2A1 : COM2B1)))))
#endif //defined(__AVR_ATmega8__)

#else  //Mega 1280,2560
// Arduino Mega Pins
#define digitalPinToPortReg(P) \
(((P) >= 22 && (P) <= 29) ? &PORTA : \
((((P) >= 10 && (P) <= 13) || ((P) >= 50 && (P) <= 53)) ? &PORTB : \
(((P) >= 30 && (P) <= 37) ? &PORTC : \
((((P) >= 18 && (P) <= 21) || (P) == 38) ? &PORTD : \
((((P) >= 0 && (P) <= 3) || (P) == 5) ? &PORTE : \
(((P) >= 54 && (P) <= 61) ? &PORTF : \
((((P) >= 39 && (P) <= 41) || (P) == 4) ? &PORTG : \
((((P) >= 6 && (P) <= 9) || (P) == 16 || (P) == 17) ? &PORTH : \
(((P) == 14 || (P) == 15) ? &PORTJ : \
(((P) >= 62 && (P) <= 69) ? &PORTK : &PORTL))))))))))

#define digitalPinToDDRReg(P) \
(((P) >= 22 && (P) <= 29) ? &DDRA : \
((((P) >= 10 && (P) <= 13) || ((P) >= 50 && (P) <= 53)) ? &DDRB : \
(((P) >= 30 && (P) <= 37) ? &DDRC : \
((((P) >= 18 && (P) <= 21) || (P) == 38) ? &DDRD : \
((((P) >= 0 && (P) <= 3) || (P) == 5) ? &DDRE : \
(((P) >= 54 && (P) <= 61) ? &DDRF : \
((((P) >= 39 && (P) <= 41) || (P) == 4) ? &DDRG : \
((((P) >= 6 && (P) <= 9) || (P) == 16 || (P) == 17) ? &DDRH : \
(((P) == 14 || (P) == 15) ? &DDRJ : \
(((P) >= 62 && (P) <= 69) ? &DDRK : &DDRL))))))))))

#define digitalPinToPINReg(P) \
(((P) >= 22 && (P) <= 29) ? &PINA : \
((((P) >= 10 && (P) <= 13) || ((P) >= 50 && (P) <= 53)) ? &PINB : \
(((P) >= 30 && (P) <= 37) ? &PINC : \
((((P) >= 18 && (P) <= 21) || (P) == 38) ? &PIND : \
((((P) >= 0 && (P) <= 3) || (P) == 5) ? &PINE : \
(((P) >= 54 && (P) <= 61) ? &PINF : \
((((P) >= 39 && (P) <= 41) || (P) == 4) ? &PING : \
((((P) >= 6 && (P) <= 9) || (P) == 16 || (P) == 17) ? &PINH : \
(((P) == 14 || (P) == 15) ? &PINJ : \
(((P) >= 62 && (P) <= 69) ? &PINK : &PINL))))))))))

#define __digitalPinToBit(P) \
(((P) >=  7 && (P) <=  9) ? (P) - 3 : \
(((P) >= 10 && (P) <= 13) ? (P) - 6 : \
(((P) >= 22 && (P) <= 29) ? (P) - 22 : \
(((P) >= 30 && (P) <= 37) ? 37 - (P) : \
(((P) >= 39 && (P) <= 41) ? 41 - (P) : \
(((P) >= 42 && (P) <= 49) ? 49 - (P) : \
(((P) >= 50 && (P) <= 53) ? 53 - (P) : \
(((P) >= 54 && (P) <= 61) ? (P) - 54 : \
(((P) >= 62 && (P) <= 69) ? (P) - 62 : \
(((P) == 0 || (P) == 15 || (P) == 17 || (P) == 21) ? 0 : \
(((P) == 1 || (P) == 14 || (P) == 16 || (P) == 20) ? 1 : \
(((P) == 19) ? 2 : \
(((P) == 5 || (P) == 6 || (P) == 18) ? 3 : \
(((P) == 2) ? 4 : \
(((P) == 3 || (P) == 4) ? 5 : 7)))))))))))))))

// 15 PWM
#define __digitalPinToTimer(P) \
(((P) == 13 || (P) ==  4) ? &TCCR0A : \
(((P) == 11 || (P) == 12) ? &TCCR1A : \
(((P) == 10 || (P) ==  9) ? &TCCR2A : \
(((P) ==  5 || (P) ==  2 || (P) ==  3) ? &TCCR3A : \
(((P) ==  6 || (P) ==  7 || (P) ==  8) ? &TCCR4A : \
(((P) == 46 || (P) == 45 || (P) == 44) ? &TCCR5A : 0))))))
#define __digitalPinToTimerBit(P) \
(((P) == 13) ? COM0A1 : (((P) ==  4) ? COM0B1 : \
(((P) == 11) ? COM1A1 : (((P) == 12) ? COM1B1 : \
(((P) == 10) ? COM2A1 : (((P) ==  9) ? COM2B1 : \
(((P) ==  5) ? COM3A1 : (((P) ==  2) ? COM3B1 : (((P) ==  3) ? COM3C1 : \
(((P) ==  6) ? COM4A1 : (((P) ==  7) ? COM4B1 : (((P) ==  8) ? COM4C1 : \
(((P) == 46) ? COM5A1 : (((P) == 45) ? COM5B1 : COM5C1))))))))))))))

#endif  //!(defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) )
#endif  //!defined(digitalPinToPortReg)

inline void _atomic_Write_( uint8_t* address, uint8_t p, uint8_t v) {
	if ((int)address < 0x20)     bitWrite(*address, __digitalPinToBit(p),v);
	else  {
		uint8_t register saveSreg = SREG;     
		cli();                                  
		bitWrite(*address, __digitalPinToBit(p), v);  
		SREG=saveSreg;
	}
}

#define digitalWrite(P, V) \
do {                       \
if (__builtin_constant_p(P) && __builtin_constant_p(V))   _atomic_Write_((uint8_t*) digitalPinToPortReg(P),P,V); \
else  __digitalWrite((P), (V));         \
}while (0)

#define pinMode(P, V) \
do {if (__builtin_constant_p(P) && __builtin_constant_p(V)) _atomic_Write_((uint8_t*) digitalPinToDDRReg(P),P,V); \
__pinMode((P), (V)); \
} while (0)


#if !defined(stopAnalogWrite)
#define stopAnalogWrite(P) \
do {if (__builtin_constant_p(P) ) { \
if (__digitalPinToTimer(P)) \
bitClear(*__digitalPinToTimer(P), __digitalPinToTimerBit(P)); \
} else {  \
turnOffPWM((P)); \
}} while (0)
#endif		


#define digitalRead(P) ( (int) __digitalReadFast2__((P)) )
#define __digitalReadFast2__(P) \
(__builtin_constant_p(P) ) ? ( \
( bitRead(*digitalPinToPINReg(P), __digitalPinToBit(P))) ) : \
__digitalRead((P)) 
#endif DigitalWrite_H
*/