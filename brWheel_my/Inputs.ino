/*
  Copyright 2015  Etienne Saint-Paul
  Copyright 2017  Fernando Igor  (fernandoigor [at] msn [dot] com)
  Copyright 2018-2021  Milos Rankovic (ranenbg [at] gmail [dot] com)

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

#include "debug.h"
#include <Wire.h>
#include <digitalWriteFast.h>
#include "Config.h"
#include "QuadEncoder.h"
//#include "HX711.h" // milos, commented
#include <HX711_ADC.h> // milos, credits to library creator Olav Kallhovd sept2017
#include "USBDesc.h"

//#define ENCODER_OPTIMIZE_INTERRUPTS
//#include <Encoder.h>

//--------------------------------------- Globals --------------------------------------------------------

//s16 adc_val = 0; //milos, commented since it was not used

//u8 digital_inputs_pins[] = {3,6,7,8,11,12}; // original

u8 analog_inputs_pins[] = // milos, changed to u8, from u16
{
  ACCEL_PIN,
#ifdef USE_LOAD_CELL //milos
  CLUTCH_PIN,
#else
  BRAKE_PIN,
  CLUTCH_PIN,
#endif
  HBRAKE_PIN
  //SHIFTER_X_PIN, // milos, commented
  //SHIFTER_Y_PIN // milos, commented
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
s32 nb_mes;
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
void InitLoadCell () { // milos, uncommented
  /*gLoadCell.power_down(); // milos, commented
    gLoadCell.power_up();
    u32 mil = millis();
    last_send = mil;
    while ((mil - last_send) < 100)
    {
  	mil = millis();
  	if (gLoadCell.is_ready())
  	{
  		gLoadCell.set_gain(128);
  		gLoadCell.tare(30);	//Assuming there is no weight on the scale at start up, reset the scale to 0
  		return(true);
  	}
    }*/
  LoadCell.begin(); // milos
  LoadCell.setGain(); // milos - set gain for channel A, default is 128, available is 64 (32 - for channel B only)
  LoadCell.start(2000); // milos, tare preciscion can be improved by adding a few seconds of stabilising time, (default 2000)
  LoadCell.setCalFactor(0.25 * float(LC_scaling)); // user set calibration factor (float) // milos
}
#endif

void InitInputs() {
  /*for (u8 i = 0; i < sizeof(digital_inputs_pins); i++) //milos, commented
    pinMode(digital_inputs_pins[i], INPUT_PULLUP);*/

  for (u8 i = 0; i < sizeof(analog_inputs_pins); i++)
    pinMode(analog_inputs_pins[i], INPUT);

  /*#ifdef USE_QUADRATURE_ENCODER
  	//myEnc.Init(ROTATION_MID + gCalibrator.mOffset);
    #endif*/

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
  //load_cell_channel = 0;
}

//--------------------------------------------------------------------------------------------------------

short int SHIFTREG_STATE;
//unsigned int bytesVal = 0;
u16 bytesVal_SW;		// Temporary variables for read input Shift Steering Wheel (8-bit)
u16 bytesVal_H;			// Temporary variables for read input Shift H-Shifter (Dual 8-bit)
u16 btnVal_SW = 0;
u16 btnVal_H = 0;
u8 i = 0;

#ifdef USE_SHIFT_REGISTER
void InitShiftRegister() {
  pinModeFast(SHIFTREG_CLK, OUTPUT); //milos, changed to fast
  pinMode(SHIFTREG_DATA_SW, INPUT_PULLUP); //milos, added pullup
  //pinMode(SHIFTREG_DATA_H, INPUT_PULLUP);  //milos, added pullup
  pinModeFast(SHIFTREG_PL, OUTPUT); //milos, changed to fast
  SHIFTREG_STATE = 0;
  bytesVal_SW = 0;
}
#else
void InitButtons() { // milos, added - if not using shift register, allocate some free pins for buttons
  //pinModeFast(BUTTON0, INPUT_PULLUP); //milos, A3 used for hbrake instead
  pinModeFast(BUTTON1, INPUT_PULLUP);
  pinModeFast(BUTTON2, INPUT_PULLUP);
}
#endif

void nextInputState() {
  u8 bitVal = 0;
  if (SHIFTREG_STATE > SHIFTS_NUM) {	// SINGLE SHIFT = 25 states  DUAL SHIFT = 49 states (both zero include), 33 for 16 shifts //milos, 49 for 24 shifts or 3 bytes, 65 for 32 shifts or 4 bytes
    SHIFTREG_STATE = 0;       //milos, we only read first 4bytes since those contain button values on thrustmaster wheel rims (rim sends total of 8bytes due to headroom for more buttons)
    btnVal_SW = bytesVal_SW; //milos, these were originaly designed for G29, where SW is 8bit shift register from steering wheel
    btnVal_H = bytesVal_H; //milos, these were originaly designed for G29, where H is dual 8bit (16bit) shift register from H-shifter
    bytesVal_SW = 0;
    bytesVal_H = 0;
    i = 0;
  }

  if (SHIFTREG_STATE < 2) {
    //DEBUG_SERIAL.println("PL");
    if (SHIFTREG_STATE == 0) {
      //Serial.println("PL low");
      digitalWriteFast(SHIFTREG_PL, HIGH); //milos, changed to fast, was LOW
    }
    else {
      //Serial.println("PL high");
      digitalWriteFast(SHIFTREG_PL, LOW); //milos, changed to fast, was HIGH
    }
  }
  else {
    if (SHIFTREG_STATE % 2 == 0) {
      bitVal = digitalRead(SHIFTREG_DATA_SW);
      bytesVal_SW |= (bitVal << ((16 - 1) - i)); //milos, was 8 (we read first 2 bytes)

      //bitVal = digitalReadFast(SHIFTREG_DATA_SW); //milos, was _H (commented)
      //bytesVal_H |= (bitVal << ((24 - 1) - i)); //milos, was 16 (we read 3rd byte only)
      bytesVal_H |= (bitVal << ((32 - 1) - i)); //milos, we read 3rd and 4th byte

      i++;
      digitalWriteFast(SHIFTREG_CLK, HIGH); //milos, changed to fast
      //Serial.println("CLK high"); // milos, added
    }
    else if (SHIFTREG_STATE % 2 == 1) {
      digitalWriteFast(SHIFTREG_CLK, LOW); //milos, changed to fast
      //Serial.println("CLK low"); // milos, added
    }
  }

  SHIFTREG_STATE++;
  //Serial.println(SHIFTREG_STATE); // milos, added
}

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

  //milos, my version
  for (u8 i = 0; i < 16; i++) { //milos, write 2 bytes from btnVal_SW into first 2 bytes of buttons
    bitWrite(buttons, i, bitRead(btnVal_SW, 15 - i));
  }
  for (u8 j = 0; j < 16; j++) { //milos, write 2 bytes from btnVal_H into last 2 bytes of buttons
    bitWrite(buttons, j + 16, bitRead(btnVal_H, 15 - j));
  }
  /*Serial.print(btnVal_SW, BIN); // milos
    Serial.print(" "); // milos
    Serial.println(btnVal_H, BIN); // milos*/
#else  // milos, when no shift reg, use Arduino Leonardo for 3 buttons
#ifdef USE_LOAD_CELL //milos - only available if we use a load cell
  bitWrite(buttons, 0, bitRead(digitalReadFast(BUTTON0), B0PORTBIT)); // milos, read bit4 from PORTF A3 into buttons bit0
#else //milos, when no shift reg and no load cell button0 is unavailable so we set it to 0 (unpressed)
  bitWrite(buttons, 0, 0);
#endif //end of lc
  bitWrite(buttons, 1, bitRead(digitalReadFast(BUTTON1), B1PORTBIT)); // milos, read bit1 from PORTF A4 (or bit3 from PORTB, pin14 on ProMicro) into buttons bit1
  bitWrite(buttons, 2, bitRead(digitalReadFast(BUTTON2), B2PORTBIT)); // milos, read bit0 from PORTF A5 (or bit1 from PORTB, pin15 on ProMicro) into buttons bit2
#endif //end of shift reg

  //DEBUG_SERIAL.println(btnVal_H);
  //return (~buttons & 0b00000000111111111100000111111111); // milos, added to mask of last 8 bits and some inside ones with inverting all bits
  //return (buttons & 0b00000000111111111111111111111111); // milos, added to mask of last 8 bits, since we are not reading them anyway
  //return (buttons & 0b00000000111111111100000111111111); // milos, do not update buttons for thrustmaster wheel rim identification (3 bytes read)
  //return (~buttons & 0b00000000000000001111111111111111); // milos, added to invert only first 16 bits
  return (buttons); // milos, we read all 4 bytes
}

//--------------------------------------------------------------------------------------------------------

#ifdef AVG_INPUTS //milos, includes this only if it is used
void ClearAnalogInputs () {
  for (u8 i = 0; i < sizeof(analog_inputs_pins); i++) {
    analog_inputs[i] = 0;
  }
  nb_mes = 0;
}

uint16_t ReadAnalogInputs () {
  for (u8 i = 0; i < sizeof(analog_inputs_pins); i++) {
    analog_inputs[i] += analogRead(analog_inputs_pins[i]);
  }
  nb_mes++;

  return nb_mes;
}

void AverageAnalogInputs () {
  for (u8 i = 0; i < sizeof(analog_inputs_pins); i++) {
    //analog_inputs[i] = (analog_inputs[i]) / nb_mes; //milos, commented
    analog_inputs[i] = (analog_inputs[i] << axis_shift_n_bits[i]) / nb_mes; //milos, fixed now
  }
}
#endif

