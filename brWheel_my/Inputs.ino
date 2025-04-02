/*
  Copyright 2015  Etienne Saint-Paul
  Copyright 2017  Fernando Igor  (fernandoigor [at] msn [dot] com)
  Copyright 2018-2025  Milos Rankovic (ranenbg [at] gmail [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#include "Config.h"
#include "QuadEncoder.h"
#include "debug.h"
#include "USBDesc.h"
#include <Arduino.h>
#include <Wire.h>
#include <digitalWriteFast.h>
#include <HX711_ADC.h> // milos, credits to library creator Olav Kallhovd sept2017


//--------------------------------------- Globals --------------------------------------------------------

u8 analog_inputs_pins[] = // milos, changed to u8, from u16
{
  ACCEL_PIN,
#ifdef USE_LOAD_CELL //milos
#ifndef USE_EXTRABTN
  CLUTCH_PIN,
#else // if no extra buttons
  CLUTCH_PIN
#endif // end of extra button
#else // if no lc
#ifndef USE_EXTRABTN
  BRAKE_PIN,
  CLUTCH_PIN,
#else
  BRAKE_PIN
#endif // end of extra button
#endif // end of load cell
  /*#ifdef USE_XY_SHIFTER // milos
    HBRAKE_PIN,
    SHIFTER_X_PIN, // milos
    SHIFTER_Y_PIN // milos
    #else*/
#ifndef USE_EXTRABTN
  HBRAKE_PIN
#endif // end of extra button
  //#endif // end of xy shifter
};

u8 axis_shift_n_bits[] =  // milos, changed to u8, from u16
{
  Z_AXIS_NB_BITS - ADC_NB_BITS,
#ifdef USE_LOAD_CELL //milos
  RX_AXIS_NB_BITS - ADC_NB_BITS,
#else
  Y_AXIS_NB_BITS - (ADC_NB_BITS + 4), // milos, added - we scale it to 4095, that's why +4
  RX_AXIS_NB_BITS - ADC_NB_BITS,
#endif
  RY_AXIS_NB_BITS - ADC_NB_BITS
};

#ifdef AVG_INPUTS //milos, include this only if used
s32 analog_inputs[sizeof(analog_inputs_pins)];

//u8 load_cell_channel;
s8 nb_mes; // milos, changed from s32 to s8
#endif

//----------------------------------------- Options -------------------------------------------------------

#ifdef USE_DSP56ADC16S
void ss_falling()
{
  adc_val = (spi_buffer[0] << 8) + spi_buffer[1];
  spi_bp = 0;
}

ISR(SPI_STC_vect)
{
  spi_buffer[spi_bp] = SPDR;
  if (spi_bp < 3)
    spi_bp++;
}
#endif


//--------------------------------------------------------------------------------------------------------

#ifdef USE_DSP56ADC16S
void InitADC () {
  pinMode(ADC_PIN_CLKIN, OUTPUT);
  pinMode(ADC_PIN_FSO, INPUT);
  pinMode(ADC_PIN_SDO, INPUT);
  pinMode(ADC_PIN_SCO, INPUT);

  SPCR = _BV(SPE);		// Enable SPI as slave
  SPCR |= _BV(SPIE);		// turn on SPI interrupts
  TCCR3A = 0b10100000;
  TCCR3B = 0b00010001;
  ICR3 = 8;
  OCR3A = 4;									// 1 MHz clock for the DSP56ADC16S
  attachInterrupt(0, ss_falling, FALLING);
}
#endif

#ifdef USE_LOAD_CELL
void InitLoadCell () { // milos, added
  LoadCell.begin(); // milos
  LoadCell.setGain(); // milos - set gain for channel A, default is 128, available is 64 (32 - for channel B only)
  LoadCell.start(2000); // milos, tare preciscion can be improved by adding a few seconds of stabilising time, (default 2000)
  LoadCell.setCalFactor(0.25 * float(LC_scaling)); // user set calibration factor (float) // milos
}
#endif

void InitInputs() {
  //analogReference(INTERNAL); // sets 2.56V on AREF pin for Leonardo or Micro, can be EXTERNAL also
  for (u8 i = 0; i < sizeof(analog_inputs_pins); i++) {
    pinMode(analog_inputs_pins[i], INPUT);
  }
#ifdef USE_XY_SHIFTER
#ifndef USE_PROMICRO
  pinMode(SHIFTER_X_PIN, INPUT);
  pinMode(SHIFTER_Y_PIN, INPUT);
#endif
#endif
#ifdef USE_SHIFT_REGISTER
  InitShiftRegister();
#else
  InitButtons();
#endif

#ifdef USE_DSP56ADC16S
  InitADC();
#endif

#ifdef USE_LOAD_CELL
  InitLoadCell(); // milos, uncommented
#endif

#ifdef AVG_INPUTS //milos, include this only if used
  nb_mes = 0;
#endif
  // milos, re-map center button to TX pin (for the case where: no optical encoder, use as5600, use center button)
#ifndef USE_QUADRATURE_ENCODER
#ifdef USE_AS5600
#ifdef USE_CENTERBTN
  pinMode(QUAD_ENC_PIN_B, INPUT_PULLUP); // milos, prevent false trigerring
  EnableInterrupt(CORE_PIN1_INT, RISING); // digital interrupt on pin TX rising edge
#endif // end of centerbtn
#endif // end of as5600
#endif // end of quad enc
}

//--------------------------------------------------------------------------------------------------------

short int SHIFTREG_STATE;
//u16 bytesVal_SW;		// Temporary variables for read input Shift Steering Wheel (8-bit)
//u16 bytesVal_H;			// Temporary variables for read input Shift H-Shifter (Dual 8-bit)
//u16 btnVal_SW; // milos, commented
//u16 btnVal_H; // milos, commented
u32 bytesVal_SHR; // milos, added - 32bit shift register input buffer for buttons
u8 i;
u8 bitVal; // milos, added - current bit readout from shift register

#ifdef USE_SHIFT_REGISTER
void InitShiftRegister() {
  pinModeFast(SHIFTREG_CLK, OUTPUT); // milos, changed to fast
  pinModeFast(SHIFTREG_DATA_SW, INPUT); // milos, changed to fast
  //pinMode(SHIFTREG_DATA_H, INPUT);  // milos, commented out
  pinModeFast(SHIFTREG_PL, OUTPUT); // milos, changed to fast
  SHIFTREG_STATE = 0;
  //bytesVal_SW = 0; // milos, commented
  //bytesVal_H = 0;
  //btnVal_SW = 0;
  //btnVal_H = 0;
  bytesVal_SHR = 0; // milos, added
  i = 0;
  bitVal = 0; // milos, added
}
#else // no shift reg
void InitButtons() { // milos, added - if not using shift register, allocate some free pins for buttons
  pinMode(BUTTON0, INPUT_PULLUP);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
#ifndef USE_PROMICRO
  pinMode(BUTTON3, INPUT_PULLUP); // for Leonardo and Micro, we can use button3 even with z-index
#else // if use proMicro
#ifndef USE_ZINDEX
  pinMode(BUTTON3, INPUT_PULLUP); // on proMicro only available if we do not use z-index
#endif // end of z-index
#endif // end of proMicro
#ifndef USE_BTNMATRIX
  pinMode(BUTTON4, INPUT_PULLUP);
  pinMode(BUTTON5, INPUT_PULLUP);
  pinMode(BUTTON6, INPUT_PULLUP);
#ifndef USE_LOAD_CELL
#ifndef USE_TWOFFBAXIS
  pinMode(BUTTON7, INPUT_PULLUP);
#endif // end of 2 ffb axis
#endif // end of load cell
#else // if we use button matrix
  pinModeFast(BUTTON4, OUTPUT);
  pinModeFast(BUTTON5, OUTPUT);
  pinModeFast(BUTTON6, OUTPUT);
  setMatrixRow(BUTTON4, HIGH);
  setMatrixRow(BUTTON5, HIGH);
  setMatrixRow(BUTTON6, HIGH);
#ifndef USE_LOAD_CELL
  pinModeFast(BUTTON7, OUTPUT);
  setMatrixRow(BUTTON7, HIGH);
#endif // end of load cell
#endif // end of button matrix
#ifdef USE_EXTRABTN
  pinMode(BUTTON8, INPUT_PULLUP);
  pinMode(BUTTON9, INPUT_PULLUP);
#endif // end of extra button
}
#endif // end of shift reg

#ifdef USE_SHIFT_REGISTER
void nextInputState() {

  bitVal = 0;
  if (SHIFTREG_STATE > SHIFTS_NUM) {	// SINGLE SHIFT = 25 states  DUAL SHIFT = 49 states (both zero include), 33 for 16 shifts // milos, 49 for 24 shifts or 3 bytes, 65 for 32 shifts or 4 bytes
    SHIFTREG_STATE = 0;       // milos, we only read first 4 bytes since those contain button values on thrustmaster wheel rims (rim sends total of 8 bytes due to headroom for more buttons)
    //btnVal_SW = bytesVal_SW; // milos, these were originaly designed for G29, where SW is 8bit shift register from steering wheel
    //btnVal_H = bytesVal_H; // milos, these were originaly designed for G29, where H is dual 8bit (16bit) shift register from H-shifter
    //bytesVal_SW = 0;
    //bytesVal_H = 0;
    bytesVal_SHR = 0; // milos, added - 32bit shift register
    i = 0;
  }
  if (SHIFTREG_STATE < 2) {
    if (SHIFTREG_STATE == 0) {
#ifndef USE_SN74ALS166N // milos, default
      digitalWriteFast(SHIFTREG_PL, HIGH); // milos, nano button box protocol)
#else // milos, with a shift register chips(s)
      digitalWriteFast(SHIFTREG_PL, LOW); // milos, parralel loading is done on LOW
      digitalWriteFast(SHIFTREG_CLK, HIGH); // milos, when we have a rising edge of clock
      digitalWriteFast(SHIFTREG_CLK, LOW); // milos
#endif // end of sn74
    }
    if (SHIFTREG_STATE == 1) {
#ifndef USE_SN74ALS166N // milos, default
      digitalWriteFast(SHIFTREG_PL, LOW); // milos, nano button box protocol
#else // milos, with a shift register chips(s)
      digitalWriteFast(SHIFTREG_PL, HIGH); // milos, serial loading is done on HIGH (data from other chips stacked chips)
#endif // end of sn74
    }
  } else {
    if (SHIFTREG_STATE % 2 == 0) {
      bitVal = bitRead(digitalReadFast(SHIFTREG_DATA_SW), 7); // milos, faster reading (bit7 of PIND register is pin D6)
      #ifdef USE_SN74ALS166N
      bitVal = !bitVal; // milos, invert the buttons
      #endif // end of sn74
      //if (i < 16) bytesVal_SW |= (bitVal << ((16 - 1) - i)); //milos, was 8 (we read first 2 bytes) // milos, commented old
      //if (i < 16) bitWrite(bytesVal_SW, i, bitVal); // milos, added

      //bitVal = digitalRead(SHIFTREG_DATA_H); //milos, commented
      //if (i > 15) bytesVal_H |= (bitVal << ((32 - 1) - i)); //milos, we read 3rd and 4th byte // milos, commented old
      //if (i > 15) bitWrite(bytesVal_H, i - 16, bitVal); // milos, added

      if (i < 32) bitWrite(bytesVal_SHR, i, bitVal); // milos, added
      i++;
      digitalWriteFast(SHIFTREG_CLK, HIGH); // milos, changed to fast, was HIGH
    } else if (SHIFTREG_STATE % 2 == 1) {
      digitalWriteFast(SHIFTREG_CLK, LOW); // milos, changed to fast, was LOW
    }
  }
  SHIFTREG_STATE++;
}
#endif

u32 readInputButtons() {
  u32 buttons = 0;
#ifdef USE_SHIFT_REGISTER // milos, Arduino Nano as a separate 16 button box
  //milos, commented original function
  /*u32 bmask = 1;
    for (u8 i = 0; i < 16; i++) //milos, was 8
    {
    if (((btnVal_SW >> i) & 1) == 1)
      buttons |= bmask;
    bmask <<= 1;

    }
    //DEBUG_SERIAL.print(btnVal_SW);
    //DEBUG_SERIAL.print(" ");
    //for (u8 i = 0; i < 24; i++) //milos, was 16
    for (u8 i = 0; i < 32; i++) //milos, expand to all 32buttons
    {
    if (((btnVal_H >> i) & 1) == 1)
      buttons |= bmask;
    bmask <<= 1;
    }*/

  // milos, my version - write 2B from bytesVal_SW into first 2B of buttons and 2B from btnVal_H into last 2B of buttons
  /*for (u8 i = 0; i < 16; i++) {
    bitWrite(buttons, i, bitRead(bytesVal_SW, i));
    bitWrite(buttons, i + 16, bitRead(bytesVal_H, i));
    }*/

  //milos, my version - write bytesVal_SHR into buttons
  /*for (u8 j = 0; j < 32; j++) {
    bitWrite(buttons, j, bitRead(bytesVal_SHR, j));
    }*/
  buttons = bytesVal_SHR;

#else  // milos, when no shift reg, use Arduino Leonardo for 3 or 4 buttons
#ifndef USE_BTNMATRIX // milos, added - read all available buttons only if we are not using button matrix
  // milos, here we define button mappings (link between arduino pins and buttons defined in HID)
  bitWrite(buttons, 0, readSingleButton(0)); // milos, first 0 is button bit in HID, second 0 in brackets is associated to arduino pin - see config.h)
  bitWrite(buttons, 1, readSingleButton(1));
  bitWrite(buttons, 2, readSingleButton(2));
  //------------- milos, start button3 case
#ifdef USE_PROMICRO // milos, for proMicro button3 on D3 is available only if not using zindex and i2C devices
#ifdef USE_ZINDEX
  // milos, we can not have button3 for proMicro when we use z-index encoder
  bitWrite(buttons, 3, 0);
#endif // end of z-index
#ifdef USE_AS5600
  // milos, we can not have button3 for proMicro when we use AS5600 (i2C)
  bitWrite(buttons, 3, 0);
#endif // end of as5600
#ifdef USE_ADS1015
  // milos, we can not have button3 for proMicro when we use ADS1015 (i2C)
  bitWrite(buttons, 3, 0);
#endif // end of ads1015
#ifdef USE_MCP4725
  // milos, we can not have button3 for proMicro when we use MCP4725 (i2C)
  bitWrite(buttons, 3, 0);
#endif // end of mcp4725
#ifndef USE_ZINDEX
#ifndef USE_AS5600
#ifndef USE_ADS1015
#ifndef USE_MCP4725
  bitWrite(buttons, 3, readSingleButton(3));   // milos, we can have button3 on D3 on proMicro only if nothing is using pin D3
#endif // end of z-index
#endif // end of as5600
#endif // end of ads1015
#endif // end of mcp4725
#else // milos, if we use Leonardo or Micro we can have button3 on D12 even with z-index and i2C devices
  bitWrite(buttons, 3, readSingleButton(3));
#endif // end of pro micro
  bitWrite(buttons, 4, readSingleButton(4));
  bitWrite(buttons, 5, readSingleButton(5));
  bitWrite(buttons, 6, readSingleButton(6));
#ifndef USE_LOAD_CELL // milos, when no load cell
#ifndef USE_AS5600 // milos, when not using as5600 we can have button7 at pin D5
#ifndef USE_TWOFFBAXIS // milos, when not using 2 FFB axis, 2nd PWM channel
  bitWrite(buttons, 7, readSingleButton(7));
#else // with 2ffb axis, D5 is used by timer3 for PWM output
  bitWrite(buttons, 7, 0); // milos, we don't have button7
#endif // end of 2 ffb axis
#else // milos, if we use mag encoder and hat switch on proMicro
#ifdef USE_PROMICRO
#ifdef USE_HATSWITCH
  bitWrite(buttons, 3, readSingleButton(7)); // milos, we need to remap D6 to button3 in order for hat switch left to work
  bitWrite(buttons, 7, 0); // milos, we don't have button7 (pin D5) in that case
#endif // end of hat swich
#else // for leonardo/micro with hat switch
  bitWrite(buttons, 7, readSingleButton(7)); // milos, button7 is on pin D5 for leonardo/micro with hat switch
#endif // end of proMicro
#endif // end of as5600
#else // milos, with load cell
  bitWrite(buttons, 7, 0); // milos, button7 (pin D5) is unavailable on all boards if we use load cell
#endif // end of 

#ifdef USE_EXTRABTN
  bitWrite(buttons, 8, readSingleButton(8));
  bitWrite(buttons, 9, readSingleButton(9));
#endif // end of extra button
#else // do matrix button readout
  // buttons 0-3 of are columns j
  // buttons 4-7 of are rows i
  // Matrix element is Bij
  //     D4  A4  A5  D12
  // D6 |b11 b12 b13 b14|
  // D7 |b21 b22 b23 b24|
  // D8 |b31 b32 b33 b34|
  // D5 |b41 b42 b43 b44|
  for (uint8_t i = 0; i < 4; i++) { // rows (along X), we set each row high, one at a time
    setMatrixRow (i, LOW);
    for (uint8_t j = 0; j < 4; j++) { // columns (along Y), read each button from that row by scanning over columns
      bitWrite(buttons, i * 4 + j, readSingleButton(j));
    }
    setMatrixRow (i, HIGH);
  }
#endif // end of button matrix
#endif // end of shift reg

#ifdef USE_HATSWITCH // milos, added
  buttons = decodeHat(buttons); // milos, decodes hat switch values into only 1st 4 buttons (button0-up, button1-right, button2-down, button3-left)
#else
  buttons = buttons << 4; // milos, bitshift to the left 4 bits to skip updating hat switch
#endif // end of hat switch

  return (buttons); // milos, we send all 4 bytes
}

#ifndef USE_SHIFT_REGISTER
bool readSingleButton (uint8_t i) { // milos, added
  bool temp;
  if (i == 0) {
    temp = !bitRead(digitalReadFast(BUTTON0), B0PORTBIT); // milos, read bit4 from PINF A3 (or bit4 from PIND D4 when no LC, or bit6 from PINC D5 on leonardo/micro when XY shifter) into buttons bit0
  } else if (i == 1) {
    temp = !bitRead(digitalReadFast(BUTTON1), B1PORTBIT); // milos, read bit1 from PINF A4 (or bit3 from PINB D14 on ProMicro) into buttons bit1
  } else if (i == 2) {
    temp = !bitRead(digitalReadFast(BUTTON2), B2PORTBIT); // milos, read bit0 from PINF A5 (or bit1 from PINB D15 on ProMicro) into buttons bit2
  } else if (i == 3) {
    temp = !bitRead(digitalReadFast(BUTTON3), B3PORTBIT); // milos, read bit6 from PIND D12 into buttons bit3
  } else if (i == 4) {
    temp = !bitRead(digitalReadFast(BUTTON4), B4PORTBIT); // milos, read bit7 from PIND D6 into buttons bit4
  } else if (i == 5) {
    temp = !bitRead(digitalReadFast(BUTTON5), B5PORTBIT); // milos, read bit6 from PINE D7 into buttons bit5
  } else if (i == 6) {
    temp = !bitRead(digitalReadFast(BUTTON6), B6PORTBIT); // milos, read bit4 from PINB D8 into buttons bit6
#ifndef USE_LOAD_CELL // milos, save some memory space if this button is unavailable
  } else if (i == 7) {
    temp = !bitRead(digitalReadFast(BUTTON7), B7PORTBIT); // milos, read bit6 from PINC D5 into buttons bit7
#endif // end of load cell
#ifdef USE_EXTRABTN // milos, if enabled we have 2 more extra digital buttons
  } else if (i == 8) {
    temp = !bitRead(digitalReadFast(BUTTON8), B8PORTBIT); // milos, read bit5 from PINF A2 into buttons bit8
  } else if (i == 9) {
    temp = !bitRead(digitalReadFast(BUTTON9), B9PORTBIT); // milos, read bit4 from PINF A3 into buttons bit9
#endif // end of extra button
  } else {
    temp = false;
  }
  return temp;
}
#endif // end of shift register

#ifdef USE_BTNMATRIX
void setMatrixRow (uint8_t j, uint8_t k) { // milos, added
  if (j == 0) {
    digitalWriteFast(BUTTON4, k);
  } else if (j == 1) {
    digitalWriteFast(BUTTON5, k);
  } else if (j == 2) {
    digitalWriteFast(BUTTON6, k);
  } else if (j == 3) {
    digitalWriteFast(BUTTON7, k);
  }
}
#endif // end of button matrix

//--------------------------------------------------------------------------------------------------------

#ifdef AVG_INPUTS //milos, include this only if it is used
void ClearAnalogInputs() {
  for (u8 i = 0; i < sizeof(analog_inputs_pins); i++) {
    analog_inputs[i] = 0;
  }
  nb_mes = 0;
}

void ReadAnalogInputs() { // milos, changed to void from uint16_t
  for (u8 i = 0; i < sizeof(analog_inputs_pins); i++) {
    analog_inputs[i] += analogRead(analog_inputs_pins[i]);
  }
  nb_mes++;

  //return nb_mes; // milos, commented out
}

void AverageAnalogInputs() {
  for (u8 i = 0; i < sizeof(analog_inputs_pins); i++) {
    analog_inputs[i] = (analog_inputs[i] << axis_shift_n_bits[i]) / nb_mes; //milos, fixed
  }
}
#endif
