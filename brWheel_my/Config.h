#ifndef _CONFIG_H_
#define _CONFIG_H_


//------------------------------------- Options ----------------------------------------------------------

//#define USE_VNH5019				// Pololu dual 24V DC motor drive
//#define USE_SM_RS485				// Granite devices simple motion protocol (not implemented yet)
//#define USE_LCD					// milos, LCD via i2c (not implemented yet)
//#define USE_ADS1105       // milos, uncomment for 12bit pedals (commented is 10bit from arduino inputs), can not be used with AVG_INPUTS
//#define USE_DSP56ADC16S			// 16 bits Stereo ADC (milos, can not be used if USE_SHIFT_REGISTER is uncommented)
#define USE_QUADRATURE_ENCODER		// Position Quadrature encoder
#define USE_ZINDEX          // milos, use Z-index encoder channel (warning, can not be used with USE_ADS1105 or USE_MCP4725)
//#define USE_LOAD_CELL				// Load cell shield // milos, new library for LC
//#define USE_SHIFT_REGISTER			// 8-bit Parallel-load shift registers G27 board steering wheel (milos, this one is modified for 16 buttons)
//#define USE_DUAL_SHIFT_REGISTER		// Dual 8-bit Parallel-load shift registers G27 board shifter  (milos, not available curently)
//#define USE_HATSWITCH        //milos, uncomment to use first 4 buttons for hat switch instead (can not be used if no load cell or shift register)
#define USE_BTNMATRIX        //milos, uncomment to use 8 pins as a 4x4 button matrix for total of 16 buttons (can not be used with USE_LOAD_CELL)
//#define AVG_INPUTS        // milos, uncomment this to use averaging of arduino analog inputs (if readings can be done faster than CONTROL_PERIOD)
#define USE_AUTOCALIB        // milos, uncomment this to use autocalibration for pedal axis
//#define USE_MCP4725      // milos, 12bit DAC (0-5V), uncomment to enable output of FFB signal as DAC voltage output
//#define USE_PROMICRO    // milos, uncomment if you are using Arduino ProMicro board (leave commented for Leonardo or Micro variants)
#define USE_EEPROM // milos, uncomment this to enable loading/saving settings from EEPROM

#define CALIBRATE_AT_INIT	0 //milos, was 1

//------------------------------------- Pins -------------------------------------------------------------

//#define LED_PIN				12
//#define	LCSYNC_LED_PIN		12
//#define	SYNC_LED_PIN		12 //milos, USB polling clock

//milos, added - ffb clip LED indicator
#ifndef USE_PROMICRO
#define  FFBCLIP_LED_PIN    13 // for Leonardo and Micro
#else  // of USE_PROMICRO
#ifndef USE_MCP4725 // milos, we can only use it if DAC is not using i2C pins 2,3
#ifndef USE_ADS1105 // milos, we can only use it if ADS1105 is not using i2C pins 2,3
#define  FFBCLIP_LED_PIN    3 // for ProMicro and no USE_ADS1105 or USE_MCP4725
#else
#define  FFBCLIP_LED_PIN    15 // for ProMicro and no USE_ADS1105
#endif // of USE_ADS1105
#else // no USE_MCP4725
#define  FFBCLIP_LED_PIN    15 // for ProMicro and no USE_MCP4725
#endif // of USE_MCP4725
#endif // of USE_PROMICRO

#define ACCEL_PIN			A0
#ifdef USE_LOAD_CELL //milos
#define CLUTCH_PIN		A1
#define HBRAKE_PIN    A2
#else
#define BRAKE_PIN     A1
#define CLUTCH_PIN    A2
#define HBRAKE_PIN    A3
#endif
//#define SHIFTER_X_PIN		A4 // milos, commented
//#define SHIFTER_Y_PIN		A5 // milos, commented

// milos, added - for alternate button0 options
#ifdef USE_LOAD_CELL //milos
#define BUTTON0 A3 // A3, used for button0
#define B0PORTBIT 4 // read bit4
#else
#define BUTTON0 4 // D4, used for button0
#define B0PORTBIT 4 // read bit4 of PIND
#define BUTTON7 5 // D5, used for button7
#define B7PORTBIT 6 // read bit6 of PINC
#endif

#ifndef USE_PROMICRO // milos, added - for Leonardo or Micro
#define BUTTON1 A4 // A4, used for button1 instead
#define B1PORTBIT 1 // read bit1
#define BUTTON2 A5 // A5, used for button2 instead
#define B2PORTBIT 0 // read bit0
#define BUTTON3 12 // used for button3
#define B3PORTBIT 6 // read bit6 or D12
#else // for Pro Micro
#define BUTTON1 14 // pin14, used for button1 instead
#define B1PORTBIT 3 // bit3
#define BUTTON2 15 // pin15, used for button2 instead
#define B2PORTBIT 1 // bit1
#define BUTTON3 2 // pin2, used for button3 instead
#define B3PORTBIT 1 // read bit1 of PIND
#endif

#ifdef USE_SHIFT_REGISTER //milos, added
#define SHIFTREG_PL			  8	  // PL SH/LD (Shift or Load input) // milos, was 4
#define SHIFTREG_CLK		  7 	// CLOCK 8-bit Parallel shift // milos, was 5 //my was 12, temp set to 7
#define SHIFTREG_DATA_SW	6		// DATA from Steering Wheel
//#define SHIFTREG_DATA_H		7		// DATA from Shifter H (Dual 8-bit) //milos, not in use
//#define SHIFTREG_DATA_OUT	13	// DATA Shift-Out LED (8-bit)   ###### NOT YET IMPLEMENTED ###### //milos, was 3
#define SHIFTS_NUM  33 // milos, added - defines number of shifts for shift register from the button box (depends on number of buttons we want to read, see inputs.ino)
#else //milos, if no shift reg re-alocate some pins to button4-6 instead
#define BUTTON4 6 // D6 or bit7 of PIND
#define B4PORTBIT 7 // bit7
#define BUTTON5 7 // D7 or bit6 of PINE
#define B5PORTBIT 6 // bit6
#define BUTTON6 8 // D8 or bit4 of PINB
#define B6PORTBIT 4 // bit4
#endif //end of shift reg

#ifdef USE_DSP56ADC16S
#define ADC_PIN_CLKIN		5
#define ADC_PIN_FSO			3//10	// Hack : This pin has been connected to mcu SS
#define ADC_PIN_SDO			13		// Hack : This pin has been connected to ICSP MOSI
#define ADC_PIN_SCO			11		// Hack : This pin has been connected to ICSP SCK
#define PIN_MISO			  12		// Hack : This pin has been connected to ICSP MISO
#endif

#define PWM_PIN_L     9 // milos, left PWM pin
#define PWM_PIN_R     10 // milos, right PWM pin
#ifndef USE_PROMICRO // milos, added - for Leonardo or Micro
#define DIR_PIN       11 // milos, PWM direction pin: 0V-left (negative force), 5V-right (positive force)
#else // for Pro Micro
#define DIR_PIN       16 // pin16 
#endif

#define ACCEL_INPUT 0
#ifdef USE_LOAD_CELL //milos
#define CLUTCH_INPUT 1
#define HBRAKE_INPUT 2
#else
#define BRAKE_INPUT 1
#define CLUTCH_INPUT 2
#define HBRAKE_INPUT 3
#endif
//#define SHIFTER_X_INPUT 3 // milos, commented
//#define SHIFTER_Y_INPUT 4 // milos, commented

uint8_t LC_scaling; // milos, load cell scaling factor (affects brake pressure, but depends on your load cell's maximum specified load)
// usage:   1 : min value, not recommended due to resolution loss
//          4 : the most sensitive and a very light brake (1:1 reading from 24bit ADC)
//         45 : ok for me, I'm using a load cell for human body weight scales, it has 75kg max load
//        128 : and higher makes the brake very hard, be careful not to phisicaly damage your load cell by applying too much force on it
//        255 : max value, extremely stiff

//------------------------------------- Helpers -----------------------------------------------------

//#define	LCSYNC_LED_HIGH()		digitalWriteFast(LCSYNC_LED_PIN,HIGH)
//#define	LCSYNC_LED_LOW()		digitalWriteFast(LCSYNC_LED_PIN,LOW)
//#define	SYNC_LED_HIGH()			digitalWriteFast(SYNC_LED_PIN,HIGH)
//#define	SYNC_LED_LOW()			digitalWriteFast(SYNC_LED_PIN,LOW)

//------------------------------------- EEPROM Config -----------------------------------------------------

#define PARAM_ADDR_VERSION			 0x00 //milos, firmware version
#define PARAM_ADDR_OFFSET        0x02 //milos, Z-index offset
#define PARAM_ADDR_ROTATION_DEG  0x06 //milos, rotation degrees
#define PARAM_ADDR_GEN_GAIN      0x08 //milos, general
#define PARAM_ADDR_DMP_GAIN      0x09 //milos, damper
#define PARAM_ADDR_FRC_GAIN      0x0A //milos, friction
#define PARAM_ADDR_CNT_GAIN      0x0B //milos, constant
#define PARAM_ADDR_PER_GAIN      0x0C //milos, periodic
#define PARAM_ADDR_SPR_GAIN      0x0D //milos, spring
#define PARAM_ADDR_INR_GAIN      0x0E //milos, inertia
#define PARAM_ADDR_CTS_GAIN      0x0F //milos, centering spring
#define PARAM_ADDR_STP_GAIN      0x10 //milos, end stop
#define PARAM_ADDR_MIN_TORQ      0x11 //milos, min torque
#define PARAM_ADDR_MAX_TORQ      0x13 //milos, max torque
#define PARAM_ADDR_MAX_DAC       0x15 //milos, max DAC value
#define PARAM_ADDR_BRK_PRES      0x17 //milos, max brake pressure
#define PARAM_ADDR_DSK_EFFC      0x18 //milos, desktop effects (byte contents is in effstate)
#define PARAM_ADDR_ENC_CPR       0x19 //milos, encoder CPR
#define PARAM_ADDR_PWM_SET       0x1D //milos, PWM settings and frequency (byte contents is in pwmstate)

#define VERSION		0xB4 // milos, this is my version (previous was 8)

#define GetParam(m_offset,m_data)	getParam((m_offset),(u8*)&(m_data),sizeof(m_data))
#define SetParam(m_offset,m_data)	setParam((m_offset),(u8*)&(m_data),sizeof(m_data))

//------------------------------------- Main Config -----------------------------------------------------

//#define CONTROL_FPS		500 //milos, commented
#define CONTROL_PERIOD	2000 //milos, original 2000 (us), be careful since this defines ffb calculation rate (min is 1000us for max 1000Hz ffb calc rate, but 16MHz clock is not fast enough)
//#define SEND_PERIOD		4000 //milos, commented
#define CONFIG_SERIAL_PERIOD 10000 // milos, original 50000 (us)

//------------------------------------- FFB Config -----------------------------------------------------

//milos, these are now loaded from EEPROM
u8 effstate; // = 0b00000001; //milos, added - turn on/off desktop effects through serial interface
//bit0-autocentering spring, bit1-damper, bit2-inertia, bit3-friction, bit4-ffb monitor (sends ffb signal data to com port), bits 5-7 are unused
u8 pwmstate; // =0b00000101; //milos, added - configures the PWM settings
//bit0-phase correct (or fast mode), bit1-dir enabled, bits 2-5 are frequency select, bits 6-7 are unused

//milos, changed these from f32 to u8 (loaded from EEPROM)
u8 configGeneralGain;  // = 1.0f;  //was 1.0f
u8 configDamperGain; // = 1.0f;		//was 0.5f
u8 configFrictionGain; // = 1.0f;	//was 0.5f
u8 configConstantGain; // = 1.0f;	//was 1.0f
u8 configPeriodicGain; // = 1.0f;	//was 1.0f
u8 configSpringGain; // = 1.0f;	//was 1.0f
u8 configInertiaGain; // = 1.0f;	//was 1.0f
u8 configCenterGain; // = 0.7f;	//was 0.7f
u8 configStopGain; // = 1.0f;	//was 1.0f

// milos, here we set the PWM resolution and frequency per channel (loaded from EEPROM)
// there are 2 PWM channels - one for each direction, so the actual FFB resolution is doubled
// Set 'TOP' for PWM resolution.  Assumes 16 MHz clock.
// fast PWM mode (no phase correction), number in brackets is selection index (see pwmset.ino)
// const unsigned int TOP = 0x7FFF; // 15-bit resolution,   488 Hz PWM (9)
// const unsigned int TOP = 0x7FFF; // 20000 pwm steps,     800 Hz PWM (8)
// const unsigned int TOP = 0x3FFF; // 14-bit resolution,   976 Hz PWM (7)
// const unsigned int TOP = 0x1FFF; // 10000 pwm steps,    1600 Hz PWM (6)
// const unsigned int TOP = 0x1FFF; // 13-bit resolution,  1953 Hz PWM (5)
// const unsigned int TOP = 0x0FFF; // 12-bit resolution,  3907 Hz PWM (4)
// const unsigned int TOP = 0x07FF; // 11-bit resolution,  7812 Hz PWM (3)
// const unsigned int TOP = 0x03FF; // 10-bit resolution, 15624 Hz PWM (2)
// const unsigned int TOP = 0x0320; // 800 pwm steps,     20000 Hz PWM (1)
// const unsigned int TOP = 0x01FF; // 9-bit resolution,  31311 Hz PWM (0)
// milos, the downside of using fast top PWM mode is that when you set pwm to 0, it is actualy not true zero, there still is a tiny 100ns pulse on both chanells at the same time apearing randomly
// this might be a trouble for some very powerful H-bridges if you use PWM+- mode, but on most it will be just fine
// in PWM+dir mode it does not matter since only one channel of PWM is used, dir pin is not on the same pin as the second PWM channel which is unused in dir mode

// milos, added - here are available TOP selections (defines PWM resolution and frequency)
uint16_t PWMtops [10] =
{
  400,
  800,
  1000,
  2000,
  4000,
  5000,
  10000,
  16383,
  20000,
  32767
};

uint16_t TOP; // milos, loaded from EEPROM
int16_t MM_MIN_MOTOR_TORQUE; // milos, loaded from EEPROM
int16_t MM_MAX_MOTOR_TORQUE; // milos, loaded from EEPROM
uint16_t MAX_DAC; // milos, loaded from EEPROM

uint16_t calcTOP(byte value) { // milos, added - function which returns TOP value from pwmstate byte
#ifndef USE_MCP4725
  byte index = 0b00000000; // index of frequency and PWM resolution selection (0-7)
  for (uint8_t i = 0; i < 4; i++) {
    bitWrite(index, i, bitRead(value, i + 2)); //milos, decode bits2-5 from pwmstate byte into index
  }
  if (bitRead(value, 0)) { // if phase correct PWM mode (pwmstate bit0=1)
    return (PWMtops[index] / 2);
  } else { // if fast PWM mode (pwmstate bit0=0)
    return (PWMtops[index]);
  }
#else
  return (MAX_DAC);
#endif
}

// arduino's map function work only up to range of int16_t variable (-32k,32k)
// I wrote mine, which can handle 32bit variables or int32_t
int32_t myMap (int32_t value, int32_t x0, int32_t x1, int32_t y0, int32_t y1) {
  return (y0 + value * (y1 - y0) / (x1 - x0));
}

f32 FFB_bal; // milos, FFB balance slider
f32 L_bal; // milos, left PWM balance multiplier
f32 R_bal; // milos, right PWM balance multiplier
f32 minTorquePP; //milos, added - min torque percents

boolean zIndexFound = false; //milos, added

// milos, added - function for decoding hat switch bits
uint32_t decodeHat(uint32_t inbits) {
  byte hat;
  byte dec = 0b00001111 & inbits; //milos, only take 1st 4 bits from inbits
  if (dec == 1) { //up
    hat = 1;
  } else if (dec == 2) { //right
    hat = 3;
  } else if (dec == 4) { //down
    hat = 5;
  } else if (dec == 8) { //left
    hat = 7;
  } else if (dec == 3) { //up_right
    hat = 2;
  } else if (dec == 6) { //down_right
    hat = 4;
  } else if (dec == 9) { //up_left
    hat = 8;
  } else if (dec == 12) { //down_left
    hat = 6;
  } else {
    hat = 0;
  }
  return ((inbits & 0b11111111111111111111111111110000) | (hat & 0b00001111)); // milos, put hat bits into first 4 bits of buttons and keep the rest unchanged
}

// milos, added - function for decoding 4x4 button matrix into 16 buttons
uint32_t decodeMatrix(uint32_t btpin) {
  // bits 0-3 of btpin are columns
  // bits 4-7 of btpin are rows
  // Matrix element is Bij
  //     D4  A4  A5  D12
  // D6 |b11 b12 b13 b14|
  // D7 |b21 b22 b23 b24|
  // D8 |b31 b32 b33 b34|
  // D5 |b41 b42 b43 b44|
  uint16_t matrix = 0;
  for (uint8_t i = 0; i < 4; i++) { // rows (along X)
    for (uint8_t j = 0; j < 4; j++) { // columns (along Y)
      bitWrite(matrix, i * 4 + j, bitRead(btpin, 4 + i) & bitRead(btpin, j));
    }
  }
  return ((btpin & 0b11111111111111110000000000000000) | (matrix & 0b00000000000000001111111111111111)); // milos, put matrix bits into first 16 bits of buttons and keep the rest unchanged
}

#endif // _CONFIG_H_
