#ifndef _CONFIG_H_
#define _CONFIG_H_


//------------------------------------- Options ----------------------------------------------------------

//#define USE_VNH5019				// Pololu dual 24V DC motor drive (partially inplemented)
//#define USE_SM_RS485				// Granite devices simple motion protocol (not implemented yet)
//#define USE_LCD					// milos, LCD via i2c (not implemented yet)
//#define USE_ADS1015       // milos, uncomment for 12bit pedals, commented is 10bit from arduino inputs (can not be used with AVG_INPUTS)
//#define USE_DSP56ADC16S			// 16 bits Stereo ADC (milos, can not be used with USE_SHIFT_REGISTER)
#define USE_QUADRATURE_ENCODER		// Position Quadrature encoder
#define USE_ZINDEX          // milos, use Z-index encoder channel (caution, can not be used with USE_ADS1015 or USE_MCP4725)
#define USE_LOAD_CELL				// Load cell shield // milos, new library for LC
#define USE_SHIFT_REGISTER			// 2x8-bit parallel-load shift registers G27 board steering wheel (milos, this one is modified for 16 buttons)
//#define USE_DUAL_SHIFT_REGISTER		// Dual 8-bit Parallel-load shift registers G27 board shifter  (milos, not available curently)
//#define USE_XY_SHIFTER    // milos, uncomment to use XY analog shifter (can not be used with USE_BTNMATRIX, note that for proMicro clutch and handbrake will be unavailable)
#define USE_HATSWITCH        // milos, uncomment to use first 4 buttons for hat switch instead
//#define USE_BTNMATRIX        // milos, uncomment to use 8 pins as a 4x4 button matrix for total of 16 buttons (can not be used with USE_LOAD_CELL, shift register or XY shifter)
//#define AVG_INPUTS        // milos, uncomment to use averaging of arduino analog inputs (can not be used with USE_ADS1015)
//#define USE_AUTOCALIB        // milos, uncomment to use autocalibration for pedal axis (if left commented manual calibration is enabled)
//#define USE_CENTERBTN    // milos, ucomment to assign digital input pin D2 for hardware wheel recenter to 0deg (not available when USE_ZINDEX, USE_ADS1015 or USE_MCP4725)
//#define USE_EXTRABTN    // milos, ucomment to configure analog inputs on pins A2 and A3 as a digital button inputs (2 extra buttons, note that clutch and handbrake will be unavailable)
//#define USE_MCP4725      // milos, 12bit DAC (0-5V), uncomment to enable output of FFB signal as 2ch DAC voltage output
#define USE_ANALOGFFBAXIS // milos, uncomment to enable other than encoder X-axis to be tied with FFB axis
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
#ifdef USE_LOAD_CELL // milos
#define CLUTCH_PIN		A1
#define HBRAKE_PIN    A2
#else
#define BRAKE_PIN     A1
#define CLUTCH_PIN    A2
#define HBRAKE_PIN    A3
#endif

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
#ifndef USE_XY_SHIFTER // milos, when no XY shifter buttons are available on analog pins
#define BUTTON1 A4 // A4, used for button1 instead
#define B1PORTBIT 1 // read bit1
#define BUTTON2 A5 // A5, used for button2 instead
#define B2PORTBIT 0 // read bit0
#else // if we use xy shifter
#define SHIFTER_X_PIN A4 // milos
#define SHIFTER_Y_PIN A5 // milos
#endif // end of xy shifter
#define BUTTON3 12 // used for button3
#define B3PORTBIT 6 // read bit6 or D12
#else // for Pro Micro
#define BUTTON1 14 // pin14, used for button1 instead
#define B1PORTBIT 3 // bit3
#define BUTTON2 15 // pin15, used for button2 instead
#define B2PORTBIT 1 // bit1
#ifndef USE_CENTERBTN // if not using center button
#define BUTTON3 2 // pin2, used for button3 instead
#define B3PORTBIT 1 // read bit1 of PIND
#else // if we use center button it uses the pin2
#define BUTTON3 3 // pin3, used for button3 instead
#define B3PORTBIT 0 // read bit0 of PIND
#endif // end of center button
//#define SHIFTER_X_PIN A4 // milos, they are not on proMicro pcb
//#define SHIFTER_Y_PIN A5 // milos, they are not on proMicro pcb
#endif // end of proMicro

#ifdef USE_SHIFT_REGISTER //milos, added
#define SHIFTREG_PL			  8	  // PL SH/LD (Shift or Load input) // milos, was 4
#define SHIFTREG_CLK		  7 	// CLOCK 8-bit Parallel shift // milos, was 5
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

#ifdef USE_EXTRABTN //milos, added - allocate 2 more buttons on A2, A3(at the expence of disabling clutch and handbrake)
#define BUTTON8 A2 // A2 or bit5 of PINF
#define B8PORTBIT 5 // bit5
#define BUTTON9 A3 // A3 or bit4 of PINF
#define B9PORTBIT 4 // bit4
#endif

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
#ifdef USE_XY_SHIFTER
#define SHIFTER_X_INPUT 3
#define SHIFTER_Y_INPUT 4
#endif
#else
#define BRAKE_INPUT 1
#define CLUTCH_INPUT 2
#define HBRAKE_INPUT 3
#ifdef USE_XY_SHIFTER
#define SHIFTER_X_INPUT 4
#define SHIFTER_Y_INPUT 5
#endif
#endif

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
#define PARAM_ADDR_SHFT_X0       0x1E //milos, XY shifter limit x0
#define PARAM_ADDR_SHFT_X1       0x20 //milos, XY shifter limit x1
#define PARAM_ADDR_SHFT_X2       0x22 //milos, XY shifter limit x2
#define PARAM_ADDR_SHFT_Y0       0x24 //milos, XY shifter limit y0
#define PARAM_ADDR_SHFT_Y1       0x26 //milos, XY shifter limit y1
#define PARAM_ADDR_SHFT_CFG      0x28 //milos, shifter configuration byte
#define PARAM_ADDR_ACEL_LO       0x2A //milos, accelerator pedal cal min
#define PARAM_ADDR_ACEL_HI       0x2C //milos, accelerator pedal cal max
#define PARAM_ADDR_BRAK_LO       0x2E //milos, brake pedal cal min
#define PARAM_ADDR_BRAK_HI       0x30 //milos, brake pedal cal max
#define PARAM_ADDR_CLUT_LO       0x32 //milos, clutch pedal cal min
#define PARAM_ADDR_CLUT_HI       0x34 //milos, clutch pedal cal max
#define PARAM_ADDR_HBRK_LO       0x36 //milos, hand brake pedal cal min
#define PARAM_ADDR_HBRK_HI       0x38 //milos, hand brake pedal cal max

#define VERSION		0xE8 // milos, firmware version (E6=230, E7=231, E8=232, E9=233) change this accordingly!

#define GetParam(m_offset,m_data)	getParam((m_offset),(u8*)&(m_data),sizeof(m_data))
#define SetParam(m_offset,m_data)	setParam((m_offset),(u8*)&(m_data),sizeof(m_data))

//------------------------------------- Main Config -----------------------------------------------------

//#define CONTROL_FPS		500 //milos, commented
#define CONTROL_PERIOD	2000 //milos, original 2000 (us), be careful since this defines ffb calculation rate (min is 1000us for max 1000Hz ffb calc rate, but 16MHz clock is not fast enough)
//#define SEND_PERIOD		4000 //milos, commented
#define CONFIG_SERIAL_PERIOD 10000 // milos, original 50000 (us)

//------------------------------------- FFB Config -----------------------------------------------------

// milos, these are now loaded from EEPROM
u8 effstate; // = 0b00000001; // milos, added - turn on/off desktop effects through serial interface
// bit0-autocentering spring, bit1-damper, bit2-inertia, bit3-friction, bit4-ffb monitor (sends ffb signal data to com port), bits 5-7 are FFB axis index
// bits 5-7 define an index that sets which axis is tied to FFB axis (by default it's at X-axis where we have an optical encoder)
// index FFB-axis
// 0     X
// 1     Y
// 2     Z
// 3     RX
// 4     RY
#ifdef USE_ANALOGFFBAXIS
byte indFFBAxis(byte value) { // milos, argument should be effstate
  byte temp = 0b00000000; // index of FFB Axis index selection (0-5)
  for (uint8_t i = 0; i < 3; i++) {
    bitWrite(temp, i, bitRead(value, i + 5)); //milos, decode bits5-7 from value byte into temp
  }
  return temp;
}
#endif

u8 pwmstate; // =0b00000101; //milos, added - configures the PWM settings
// bit0-phase correct (0 is fast top), bit1-dir enabled (0 is pwm+-), bits 2-5 are frequency select, bit6-enable pwm0.50.100, bit7 is unused

// bit0 pwm_type
// 0    fast pwm
// 1    phase correct

// bit1 bit6 pwm_mode
// 0    0    pwm+-
// 0    1    pwm0.50.100
// 1    0    pwm+dir
// 1    1    rcm

//if USE_MCP4725 is defined then pwmstate has a following meaning
//bits 0-5 unused, bit6-enable dac+dir (0 is dac+-), bit7-enable dac (0 is no dac output)

// bit6 pwm_type
// 0    dac+-
// 1    dac+dir

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
// fast PWM mode (for phase correct mode num of pwm steps is halved), number in brackets are array index (see PWMtops, pwm.ino, InitPWM)
// const unsigned int TOP = 0xFFFF; // 16-bit resolution,   244 Hz PWM (12)
// const unsigned int TOP = 0x9C40; // 40000 pwm steps,     400 Hz PWM (11)
// const unsigned int TOP = 0x7530; // 30000 pwm steps,     533 Hz PWM (10)
// const unsigned int TOP = 0x7FFF; // 15-bit resolution,   488 Hz PWM (9)
// const unsigned int TOP = 0x4E20; // 20000 pwm steps,     800 Hz PWM (8)
// const unsigned int TOP = 0x3FFF; // 14-bit resolution,   976 Hz PWM (7)
// const unsigned int TOP = 0x2710; // 10000 pwm steps,    1600 Hz PWM (6)
// const unsigned int TOP = 0x1FFF; // 13-bit resolution,  1953 Hz PWM (5)
// const unsigned int TOP = 0x0FFF; // 12-bit resolution,  3907 Hz PWM (4)
// const unsigned int TOP = 0x07FF; // 11-bit resolution,  7812 Hz PWM (3)
// const unsigned int TOP = 0x03FF; // 10-bit resolution, 15624 Hz PWM (2)
// const unsigned int TOP = 0x0320; // 800 pwm steps,     20000 Hz PWM (1)
// const unsigned int TOP = 0x0190; // 400 pwm steps,     40000 Hz PWM (0)
// milos, the downside of using fast top PWM mode is that when you set pwm to 0, it is actualy not true zero, there still is a tiny 100ns pulse on both chanells at the same time apearing randomly
// this might be a trouble for some very powerful H-bridges if you use PWM+- mode, but on most it will be just fine
// in PWM+dir mode it does not matter since only one channel of PWM is used, dir pin is not on the same pin as the second PWM channel which is unused in dir mode

// milos, added - available TOP selections (defines PWM resolution and frequency)
uint16_t PWMtops [13] =
{
  400,  // 0
  800,  // 1
  1000, // 2
  2000, // 3
  4000, // 4
  5000, // 5
  10000, // 6
  16383, // 7
  20000, // 8
  32767, // 9
  30000, // 10
  40000, // 11
  65535  // 12
};

uint16_t TOP; // milos, loaded from EEPROM
uint16_t MM_MIN_MOTOR_TORQUE; // milos, loaded from EEPROM
uint16_t MM_MAX_MOTOR_TORQUE; // milos, loaded from EEPROM
uint16_t MAX_DAC; // milos, loaded from EEPROM

uint16_t calcTOP(byte value) { // milos, added - function which returns TOP value from pwmstate byte
#ifndef USE_MCP4725
  byte index = 0b00000000; // index of frequency and PWM resolution selection (0-12)
  for (uint8_t i = 0; i < 4; i++) {
    bitWrite(index, i, bitRead(value, i + 2)); //milos, decode bits2-5 from pwmstate byte into index
  }
  if (bitRead(value, 0)) { // if phase correct PWM mode (pwmstate bit0=1)
    return (PWMtops[index] >> 1); // milos, divide by 2
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
f32 minTorquePP; //milos, added - min torque in percents

// milos, added - RCM pwm mode definitions
f32 RCM_min = 1.0; // minimal force RCM pulse width in ms
f32 RCM_zer = 1.5; // zero force RCM pulse width in ms
f32 RCM_max = 2.0; // maximal force RCM pulse width in ms

f32 RCMscaler (byte value) { // milos, added - scales correctly RCM pwm mode
  if (bitRead(value, 0)) { // if pwmstate bit0=1
    return 1000.0; // for phase correct pwm mode
  } else {  // if pwmstate bit0=0
    return 2000.0; // for fast pwm mode
  }
}

boolean zIndexFound = false; //milos, added - keeps track if z-index pulse from encoder was found after powerup

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

uint16_t sCal[5]; // milos, added - shifter calibration variables
// i  cal gears_if_<=
// 0  x0  1
// 1  x1  3
// 2  x2  5
// 3  y0  2,4,6,8, R
// 4  y1  N

uint8_t sConfig; // milos, added - analog XY H-shifter configuration byte
//bit0-revBtn invert,bit1-8 gears,bit2-X invert,bit3-Y invert,bit4-unused,bit5-unused,bit6-unused,bit7-unused
//bit0=1: reverse gear button inverted (for logitech G25/G27/G29/G923 H-shifters)
//bit1=1: 8 gear mode (r gear in 8th)
//bit2=1: X-axis inverted
//bit3=1: Y-axis inverted
// milos - added, function for decoding XY shifter analog values into last 8 buttons
uint32_t decodeXYshifter (uint32_t inbits, int16_t sx, int16_t sy) {
  uint32_t gears = 0; // shifter gears represented as digital buttons (1 bit for each gear)
  uint8_t revButtonBit = 0; // reverse gear button bit number (normal buttons start from bit4, bit0-bit3 are for hat switch)
  if (bitRead(sConfig, 0)) {   // if reverse gear button is inverted (logitech shifters)
    if (bitRead(inbits, revButtonBit + 4)) { // read rev gear button bit
      bitClear(inbits, revButtonBit + 4); // if 1, set rev gear bit to 0
    } else {
      bitSet(inbits, revButtonBit + 4); // if 0, set rev gear bit to 1
    }
  }
  if (sx < sCal[0] && sy >= sCal[4]) { // 1st gear
    bitSet(gears, 16);
  } else if (sx < sCal[0] && sy < sCal[3]) { // 2nd gear
    bitSet(gears, 17);
  } else if (sx >= sCal[0] && sx < sCal[1] && sy >= sCal[4]) { // 3rd gear
    bitSet(gears, 18);
  } else if (sx >= sCal[0] && sx < sCal[1] && sy < sCal[3]) { // 4th gear
    bitSet(gears, 19);
  } else if (sx >= sCal[1] && sx < sCal[2] && sy >= sCal[4]) { // 5th gear
    bitSet(gears, 20);
  } else if (sx >= sCal[1] && sx < sCal[2] && sy < sCal[3]) { // 6th gear
    if (bitRead(sConfig, 1)) { // if 8 gear shifter
      bitSet(gears, 21); // set 6th gear
    } else { // if 6 gear shifter
      if (bitRead(inbits, revButtonBit + 4)) {
        bitSet(gears, revButtonBit); // reverse gear
      } else {
        bitSet(gears, 21); // still set 6th gear
      }
    }
  } else if (sx >= sCal[2] && sy >= sCal[4]) { // 7th gear
    bitSet(gears, 22);
  } else if (sx >= sCal[2] && sy < sCal[3]) { // 8th gear
    if (bitRead(sConfig, 1)) { // if 8 gear shifter
      if (bitRead(inbits, revButtonBit + 4)) {
        bitSet(gears, revButtonBit); // reverse gear
      } else {
        bitSet(gears, 23); // set 8th gear
      }
    } else { // if 6 gear shifter
      bitSet(gears, 23); // still set 8th gear
    }
  }
  return ((inbits & 0b11110000000011111111111111101111) | (gears << 4));
}

const uint8_t avgSamples = 4; // milos, added - number of samples for averaging of arduino analog inputs
// milos, default axis calibration values depend on usage of averaging or external ADC
#ifdef AVG_INPUTS
const uint16_t maxCal = 4095;
#else // if no avg inputs
#ifdef USE_ADS1015
const uint16_t maxCal = 2047;
#else // if no ADS
const uint16_t maxCal = 1023;
#endif // end of ads
#endif // end of avg inputs

#endif // _CONFIG_H_
