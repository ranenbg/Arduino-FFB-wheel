/* Based on Encoder Library, for measuring quadrature encoded signals
  http://www.pjrc.com/teensy/td_libs_Encoder.html
  Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
  Copyright 2015  Etienne Saint-Paul  (esaintpaul [at] gameseed [dot] fr)
  Copyright 2018-2021  Milos Rankovic (ranenbg [at] gmail [dot] com)

  Version 1.4 - milos, added Z-index support
  Version 1.3 - replaced assembler code with an increment table, and deal with a third index signal directly in the interrupt
  Version 1.2 - fix -2 bug in C-only code
  Version 1.1 - expand to support boards with up to 60 interrupts
  Version 1.0 - initial release

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include <Arduino.h>
#include <digitalWriteFast.h>

cQuadEncoder gQuadEncoder;

volatile b8 gIndexFound;
volatile u8 gLastState;
volatile s32 gPosition;

//--------------------------------------------------------------------------------------------------------

//Arduino UNO
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__)
#define CORE_PIN2_INT		0
#define CORE_PIN3_INT		1

// Arduino Mega
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define CORE_PIN2_INT		0
#define CORE_PIN3_INT		1
#define CORE_PIN18_INT		5
#define CORE_PIN19_INT		4
#define CORE_PIN20_INT		3
#define CORE_PIN21_INT		2

// Arduino Leonardo
#elif defined(__AVR_ATmega32U4__) && !defined(CORE_TEENSY)
#define CORE_PIN0_INT		2
#define CORE_PIN1_INT		3
#define CORE_PIN2_INT		1
#define CORE_PIN3_INT		0
#endif

static void EnableInterrupt(u8 num, u8 type)
{
  switch (num)
  {
#if defined(EICRA) && defined(EIMSK)
    case 0:
      EICRA = (EICRA & 0xFC) | type;
      EIMSK |= 0x01;
      return;
    case 1:
      EICRA = (EICRA & 0xF3) | (type << 2);
      EIMSK |= 0x02;
      return;
    case 2:
      EICRA = (EICRA & 0xCF) | (type << 4);
      EIMSK |= 0x04;
      return;
    case 3:
      EICRA = (EICRA & 0x3F) | (type << 6);
      EIMSK |= 0x08;
      return;
#elif defined(MCUCR) && defined(GICR)
    case 0:
      MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
      GICR |= (1 << INT0);
      return;
    case 1:
      MCUCR = (MCUCR & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
      GICR |= (1 << INT1);
      return;
#elif defined(MCUCR) && defined(GIMSK)
    case 0:
      MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
      GIMSK |= (1 << INT0);
      return;
    case 1:
      MCUCR = (MCUCR & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
      GIMSK |= (1 << INT1);
      return;
#endif
#if defined(EICRB) && defined(EIMSK)
    case 4:
      EICRB = (EICRB & 0xFC) | type;
      EIMSK |= 0x10;
      return;
    case 5:
      EICRB = (EICRB & 0xF3) | (type << 2);
      EIMSK |= 0x20;
      return;
    case 6:
      EICRB = (EICRB & 0xCF) | (type << 4);
      EIMSK |= 0x40;
      return;
    case 7:
      EICRB = (EICRB & 0x3F) | (type << 6);
      EIMSK |= 0x80;
      return;
#endif
  }
}

//--------------------------------------------------------------------------------------------------------

void cQuadEncoder::Init (s32 position, b8 pullups)
{
  u8 it = pullups ? INPUT_PULLUP : INPUT;
  pinMode(QUAD_ENC_PIN_A, it);
  pinMode(QUAD_ENC_PIN_B, it);
#ifdef USE_ZINDEX // milos, added
  pinMode(QUAD_ENC_PIN_I, it);
#endif

  gIndexFound = false;
  gPosition = position;
  gLastState = 0;
  if (digitalReadFast(QUAD_ENC_PIN_A)) gLastState |= 1;
  if (digitalReadFast(QUAD_ENC_PIN_B)) gLastState |= 2;
  EnableInterrupt(CORE_PIN0_INT, CHANGE);
  EnableInterrupt(CORE_PIN1_INT, CHANGE);
  //EnableInterrupt(CORE_PIN2_INT, CHANGE); //milos, commented
  //PCMSK0 = 0x10;                // If you want to use PCINT pins...  //milos, commented
  //PCICR |= (1 << PCIE0);  //milos, commented
}

s32 cQuadEncoder::Read()
{
  noInterrupts();
  s32 pos = gPosition;
  interrupts();
  return (pos);
}

void cQuadEncoder::Write (s32 pos)
{
  noInterrupts();
  gPosition = pos;
  interrupts();
}

s8 pos_inc[] =
{
  0,		// 0 not possible
  1,		// 1
  -1,		// 2
  2,		// 3
  -1,		// 4
  0,		// 5 not possible
  -2,		// 6
  1,		// 7
  1,		// 8
  -2,		// 9
  0,		// 10 not possible
  -1,		// 11
  2,		// 12
  -1,		// 13
  1,		// 14
  0,		// 15 not possible
};

void cQuadEncoder::Update()
{
  u8 state = gLastState;
  // 	if (digitalReadFast(QUAD_ENC_PIN_A)) state |= 4;
  // 	if (digitalReadFast(QUAD_ENC_PIN_B)) state |= 8;
  u8 pd = PIND;											// Optim : change code according to the pins and the mcu used (or use the lines above)
  state |= pd & 0b1100;
  gLastState = (state >> 2);
  gPosition += pos_inc[state];
  if (gIndexFound)
    return;
  pd &= 2;
  if (pd == 0)	//digitalReadFast(QUAD_ENC_PIN_I))
    return;
#ifdef USE_ZINDEX //milos, added
  gIndexFound = true;
  zIndexFound = true;
  gPosition = ROTATION_MID;
#endif
}

ISR(INT2_vect)
{
  gQuadEncoder.Update();
}

ISR(INT3_vect)
{
  gQuadEncoder.Update();
}

