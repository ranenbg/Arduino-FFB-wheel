#ifndef _CONFIG_H_
#define _CONFIG_H_


//------------------------------------- Firmware options -------------------------------------------------

#define USE_CONFIGCDC      // milos, use virtual RS232 serial port for configuring firmware settings
//#define USE_CONFIGHID      // milos, use native USB HID protocol for configuring firmware settings (not implemented fully yet)
//#define USE_VNH5019				// Pololu dual 24V DC motor drive (partially inplemented)
//#define USE_SM_RS485				// Granite devices simple motion protocol (not implemented yet)
//#define USE_LCD					// milos, LCD via i2C (not implemented yet)
//#define USE_ADS1015       // milos, uncomment for 12bit pedals, when commented out it's 10bit from arduino inputs (can not be used with AVG_INPUTS)
//#define USE_DSP56ADC16S			// 16 bits Stereo ADC (milos, not implemented fully yet)
#define USE_QUADRATURE_ENCODER		// milos, optical quadrature encoder (you may comment it out)
//#define USE_TWOFFBAXIS        // milos, uncomment to enable 2nd FFB axis and PWM/DAC output for flight sticks (can't be used with USE_LOAD_CELL and without USE_ANALOGFFBAXIS)
//#define USE_AS5600          // milos, uncomment to enable magnetic encoder via i2C instead of optical encoder
//#define USE_TCA9548        // milos, uncomment to enable i2C multiplexer chip for using more than one AS5600 magnetic sensor via i2C (for now only used as y-axis input, must use with AS5600 and TWOFFBAXIS) 
//#define USE_ZINDEX          // milos, use Z-index encoder channel (caution, can not be used with USE_ADS1015, USE_MCP4725 or USE_AS5600)
//#define USE_LOAD_CELL				// Load cell shield // milos, new library for LC (caution can not be used with TWOFFBAXIS)
//#define USE_SHIFT_REGISTER			// 2x8-bit parallel-load shift registers G27 board steering wheel (milos, this one I modified for 16 buttons, caution can not be used with TWOFFBAXIS)
//#define USE_DUAL_SHIFT_REGISTER		// Dual 8-bit Parallel-load shift registers G27 board shifter  (milos, not available curently, use SN74ALS166N instead for 24 buttons)
//#define USE_SN74ALS166N          // milos, uncomment to use 3x8bit parralel-in serial-out shift register chips for 24 buttons, otherwise it's 16 buttons with ard nano-button box (must be used with USE_SHIFT_REGISTER)
//#define USE_XY_SHIFTER    // milos, uncomment to use XY analog shifter (can not be used with USE_BTNMATRIX, note that for proMicro clutch and handbrake will be unavailable)
//#define USE_HATSWITCH        // milos, uncomment to use first 4 buttons for hat switch (D-pad)
//#define USE_BTNMATRIX        // milos, uncomment to use 8 pins as a 4x4 button matrix for total of 16 buttons (can not be used with load cell, shift register or XY shifter)
//#define AVG_INPUTS        // milos, uncomment to use averaging of arduino analog inputs (can not be used with USE_ADS1015)
//#define USE_AUTOCALIB        // milos, uncomment to use autocalibration for pedal axis (if left commented manual calibration is enabled)
//#define USE_CENTERBTN    // milos, uncomment to assign digital input pin D2 (or TX) for hardware wheel recenter to 0deg (caution, can only be used if quad encoder usage is commented out)
//#define USE_EXTRABTN    // milos, uncomment to configure analog inputs on pins A2 and A3 as a digital button inputs (2 extra buttons, note that clutch and handbrake will be unavailable)
//#define USE_MCP4725      // milos, 12bit DAC (0-5V), uncomment to enable output of FFB signal as 2ch DAC voltage output
#define USE_ANALOGFFBAXIS // milos, uncomment to enable other than X-axis to be tied with xFFB axis (you can use analog inputs instead of digital encoders
//#define USE_PROMICRO    // milos, uncomment if you are using Arduino ProMicro board (leave commented for Leonardo or Micro variants)
#define USE_EEPROM     // milos, uncomment to enable loading/saving settings from EEPROM (if commented out, default settings will be loaded on each powerup, one needs to reconfigure firmware defautls or use GUI configuration after each powerup) 

#define CALIBRATE_AT_INIT	0 // milos, was 1

//------------------------------------- Pins -------------------------------------------------------------

//#define LED_PIN				12
//#define	LCSYNC_LED_PIN		12
//#define	SYNC_LED_PIN		12 //milos, USB polling clock

//milos, added - ffb clip LED indicator
#define FFBCLIP_LED_PIN 13 // only for leonardo/micro

#ifdef USE_PROMICRO // milos, if we use proMicro
#ifndef USE_MCP4725 // milos, we can only use it if DAC is not using i2C pins 2,3
#ifndef USE_ADS1015 // milos, we can only use it if ADS1015 is not using i2C pins 2,3
#ifndef USE_AS5600 // milos, we can only use it if AS5600 is not using i2C pin 2,3
#define FFBCLIP_LED_PIN 3 // for ProMicro if no above i2C devices
#endif // end of as5600
#endif // end of ads1015
#endif // end of mcp4725
#endif // end of promicro

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
#ifdef USE_LOAD_CELL // milos, with load cell
#define BUTTON0 A3 // A3, used for button0
#define B0PORTBIT 4 // read bit4
#else // no load cell
#define BUTTON0 4 // D4, used for button0
#define B0PORTBIT 4 // read bit4 of PIND
#define BUTTON7 5 // D5, used for button7
#define B7PORTBIT 6 // read bit6 of PINC
#endif // end of load cell

#ifdef USE_SHIFT_REGISTER // milos, added
#define SHIFTREG_PL 8 // PL SH/LD (Shift or Load input) // milos, was 4
#define SHIFTREG_CLK 7 // CLOCK 8-bit Parallel shift // milos, was 5
#define SHIFTREG_DATA_SW 6 // DATA from Steering Wheel
//#define SHIFTREG_DATA_H 7 // DATA from Shifter H (Dual 8-bit) //milos, not in use
//#define SHIFTREG_DATA_OUT 13 // DATA Shift-Out LED (8-bit)   ###### NOT YET IMPLEMENTED ###### //milos, was 3
#define SHIFTS_NUM 33 // milos, defines number of shifts (16) from the nano button box (depends on number of buttons we want to read, max is 65 for 32 buttons but we have only 24 ih hid, see inputs.ino and hid.cpp)
#ifdef USE_SN74ALS166N // milos, if we use shift register chips
#ifndef USE_XY_SHIFTER // milos, if we don't use xy shifter at the same time
#define SHIFTS_NUM 49 // milos, for 24 shifts (3x8bit shift register chips)
#endif // end of xy shifter
#endif // end of sn74
#else // milos, if no shift reg re-alocate some pins for buttons 4-6 instead
#define BUTTON4 6 // D6 or bit7 of PIND
#define B4PORTBIT 7 // bit7
#define BUTTON5 7 // D7 or bit6 of PINE
#define B5PORTBIT 6 // bit6
#define BUTTON6 8 // D8 or bit4 of PINB
#define B6PORTBIT 4 // bit4
#endif // end of shift reg

#ifndef USE_PROMICRO // milos, added - for Leonardo or Micro
#ifndef USE_XY_SHIFTER // milos, when no XY shifter - buttons are available on analog pins
#define BUTTON1 A4 // A4, used for button1 instead
#define B1PORTBIT 1 // read bit1 of PINF
#define BUTTON2 A5 // A5, used for button2 instead
#define B2PORTBIT 0 // read bit0 of PINF
#else // if we use xy shifter on leonardo or micro
#define SHIFTER_X_PIN A4 // milos
#define SHIFTER_Y_PIN A5 // milos
// milos, re-map buttons 1,2 to be available for hat switch on leonardo/micro when using xy shifter
#ifdef USE_HATSWITCH // milos, alternate button0,4 mappings because of hat switch with xy shifter
#define BUTTON0 5 // D5, used for button0
#define B0PORTBIT 6 // read bit6 of PINC
#define BUTTON4 4 // D4, used for button4
#define B4PORTBIT 4 // read bit4 of PIND
#endif // end of hat switch
#define BUTTON1 6 // D6, used for button1 instead
#define B1PORTBIT 7 // read bit7 of PIND
#define BUTTON2 7 // D7, used for button2 instead
#define B2PORTBIT 6 // read bit6 of PINE
// milos, end of button re-map
#endif // end of xy shifter
#define BUTTON3 12 // D12, used for button3 on leonardo/micro
#define B3PORTBIT 6 // read bit6 of PIND
#else // for Pro Micro
// milos, re-map buttons 0,4,7 when xy shifter is used to be available for hat switch on proMicro
#ifdef USE_HATSWITCH // milos, with hat switch
#define BUTTON0 5 // D5, used for button0
#define B0PORTBIT 6 // read bit6 of PINC
#define BUTTON4 4 // D4, used for button4
#define B4PORTBIT 4 // read bit4 of PIND
#define BUTTON7 6 // D6, used for button7
#define B7PORTBIT 7 // read bit7 of PIND
#endif // end of hat switch
#define BUTTON1 14 // D14, used for button1 instead
#define B1PORTBIT 3 // read bit3 of PINB
#define BUTTON2 15 // D15, used for button2 instead
#define B2PORTBIT 1 // read bit1 of PINB
#ifndef USE_CENTERBTN // if not using center button
#define BUTTON3 2 // D2, used for button3 instead on proMicro
#define B3PORTBIT 1 // read bit1 of PIND
#else // if we use center button it uses the pin D2
#define BUTTON3 3 // D3, used for button3 instead, we have no FFB clip LED
#define B3PORTBIT 0 // read bit0 of PIND
#endif // end of center button
//#define SHIFTER_X_PIN A4 // milos, they are not on proMicro pcb
//#define SHIFTER_Y_PIN A5 // milos, they are not on proMicro pcb
#endif // end of proMicro

#ifdef USE_EXTRABTN // milos, added - allocate 2 more buttons on A2,A3 (at the cost of disabling clutch and handbrake axis)
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
#ifdef USE_TWOFFBAXIS
#define PWM_PIN_U     11 // milos, up PWM pin
#define PWM_PIN_D     5 // milos, down PWM pin
#endif
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

#define PARAM_ADDR_FW_VERSION		 0x00 //milos, firmware version
#define PARAM_ADDR_ENC_OFFSET    0x02 //milos, Z-index offset
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

#define FIRMWARE_VERSION         0xFA // milos, firmware version (FA=250, FB=251, FC=252, FD=253)

#define GetParam(m_offset,m_data)	getParam((m_offset),(u8*)&(m_data),sizeof(m_data))
#define SetParam(m_offset,m_data)	setParam((m_offset),(u8*)&(m_data),sizeof(m_data))

//------------------------------------- Main Config -----------------------------------------------------

//#define CONTROL_FPS		500 // milos, commented out
#define CONTROL_PERIOD	2000 // milos, original 2000 (us), be careful since this defines ffb calculation rate (min is 1000us for max 1000Hz ffb calc rate, but 16MHz clock is not fast enough)
//#define SEND_PERIOD		4000 // milos, commented out
#define CONFIG_SERIAL_PERIOD 10000 // milos, original 50000 (us)

//------------------------------------- FFB/Firmware config -----------------------------------------------------

typedef struct fwOpt { // milos, added - firmware option stuct
  boolean a = false; // autocalibration (of analog axis)
  boolean b = false; // 2-ffb axis
  boolean c = false; // center button
  boolean d = false; // no optical encoder
  boolean e = false; // extra buttons
  boolean f = false; // xy shifter
  boolean g = false; // external dac (mcp4725)
  boolean h = false; // hat switch
  boolean i = false; // averaging (of analog axis)
  boolean l = false; // load cell
  boolean m = false; // proMicro pinouts
  boolean n = false; // shift register (nano button box)
  boolean p = false; // no EEPROM
  boolean r = false; // shift register (SN74ALS166N)
  boolean s = false; // exernal adc (ads1015 or ads1115)
  boolean t = false; // button matrix
  boolean u = false; // i2C multiplexer chip (tca9548), for 2nd as5600
  boolean w = false; // magnetic encoder (as5600)
  boolean x = false; // analog axis for ffb
  boolean z = false; // z-index
};

void update(fwOpt *option) { // milos, added - update firmware options from predefines above
#ifdef USE_AUTOCALIB
  option->a = true;
#endif
#ifdef USE_TWOFFBAXIS
  option->b = true;
#endif
#ifdef USE_CENTERBTN
  option->c = true;
#endif
#ifndef USE_QUADRATURE_ENCODER
  option->d = true;
#endif
#ifdef USE_EXTRABTN
  option->e = true;
#endif
#ifdef USE_XY_SHIFTER
  option->f = true;
#endif
#ifdef USE_MCP4725
  option->g = true;
#endif
#ifdef USE_HATSWITCH
  option->h = true;
#endif
#ifdef AVG_INPUTS
  option->i = true;
#endif
#ifdef USE_LOAD_CELL
  option->l = true;
#endif
#ifdef USE_PROMICRO
  option->m = true;
#endif
#ifndef USE_EEPROM
  option->p = true;
#endif
#ifdef USE_SHIFT_REGISTER
  option->n = true;
#endif
#ifdef USE_SN74ALS166N
  option->r = true;
#endif
#ifdef USE_ADS1015
  option->s = true;
#endif
#ifdef USE_BTNMATRIX
  option->t = true;
#endif
#ifdef USE_AS5600
  option->w = true;
#endif
#ifdef USE_ANALOGFFBAXIS
  option->x = true;
#endif
#ifdef USE_ZINDEX
  option->z = true;
#endif
}

// milos, these are now loaded from EEPROM
u8 effstate; // = 0b00000001; // milos, added - turn on/off desktop effects through serial interface, bit 7 is MSB
// bit0-autocentering spring, bit1-damper, bit2-inertia, bit3-friction, bit4-ffb monitor (sends ffb signal data to com port), bits 5-7 are FFB axis index
// bits 5-7 define an index that sets which axis is tied to xFFB axis (by default it's at X-axis)
// index FFB-axis
// 0     X
// 1     Y
// 2     Z
// 3     RX
// 4     RY
#ifdef USE_ANALOGFFBAXIS
byte indxFFBAxis(byte value) { // milos, argument should be effstate
  byte temp = 0b00000000; // index of FFB Axis index selection (0-5)
  for (uint8_t i = 0; i < 3; i++) {
    bitWrite(temp, i, bitRead(value, i + 5)); //milos, decode bits5-7 from value byte into temp
  }
  return temp;
}
#endif

u8 pwmstate; // =0b00000101; // milos, PWM settings configuration byte, bit7 is MSB
//---------------------
// bit0-phase correct (0 is fast pwm), bit1-dir enabled (0 is pwm+-), bits 2-5 are frequency select, bit6-enable pwm0-50-100, bit7 is unused
// bit0 pwm_type
// 0    fast pwm
// 1    phase correct

// bits2-5 define frequency index, bit2 is MSB, see firmware_info.txt for details

// bit1 bit6 pwm_mode
// 0    0    pwm+-
// 0    1    pwm0-50-100
// 1    0    pwm+dir
// 1    1    rcm
//----------------------

// milos, if USE_MCP4725 is defined then pwmstate byte has the following interpretation
//----------------
// bits 0-4 unused, bit5-enable dac0-50-100, bit6-enable dac+dir (0 is dac+-), bit7-enable dac (0 is zero dac output)

// bit7 dac_out
// 0   disabled (set to 0V or 2.5V depending on dac mode)
// 1   enabled

// bit6 bit5 dac_mode      dac_mode2 (2-ffb axis)
// 0    0    dac+-         1ch dac+- (xFFB)
// 0    1    dac0-50-100   2ch dac0-50-100
// 1    0    dac+dir       2ch dac+dir
// 1    1    none
//----------------

// milos, changed these from f32 to u8 (loaded from EEPROM)
u8 configGeneralGain;  // = 1.0f;  // was 1.0f
u8 configDamperGain; // = 1.0f;		// was 0.5f
u8 configFrictionGain; // = 1.0f;	// was 0.5f
u8 configConstantGain; // = 1.0f;	// was 1.0f
u8 configPeriodicGain; // = 1.0f;	// was 1.0f
u8 configSpringGain; // = 1.0f;	// was 1.0f
u8 configInertiaGain; // = 1.0f; // was 1.0f
u8 configCenterGain; // = 0.7f;	// was 0.7f
u8 configStopGain; // = 1.0f;	// was 1.0f

// milos, here we set the PWM resolution and frequency per channel (old, now loaded from EEPROM)
// there are 2 PWM channels - one for each direction, so the actual FFB resolution is doubled
// Set 'TOP' for PWM resolution.  Assumes 16 MHz clock.
// fast PWM mode (for phase correct mode num of PWM steps is halved), number in brackets are index for PWMtop[] array (see pwm.ino and InitPWM() function)
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
// milos, the downside of using fast PWM mode is that when you set PWM to 0, it is actualy not true zero, there is still a tiny 100ns pulse on both chanells at the same time apearing randomly
// this might be a trouble for some very powerful H-bridges if you use PWM+- mode, but on most it will be just fine
// in PWM+dir mode it does not matter since only one channel of PWM is used, dir pin is not on the same pin as the second PWM channel which is unused in dir mode

// milos, added - available TOP selection, defines PWM output resolution and frequency => affects output FFB fidelity, you want high TOP's
// I've set FFB steps to 32767 in HID descriptor, but games usualy have only up to 10000 FFB steps for force magnitude
// therefore you don't really need anything above PWMtops = 10000
// bare in mind that what you can feel in your hands will finaly depend on used motor and motor driver (good quality of digital audio signal will sound bad on shitty speakers, even with a good amp)
// drivers: AASD - great, brushless driver with FOC - good, BTS7960 - ok
// motors: brushless AC servo - great, brushless DC - good, brushed DC - ok

uint16_t PWMtops [13] =
{
  400,  // 0
  800,  // 1
  1023, // 2
  2047, // 3
  4095, // 4
  5000, // 5
  10000, // 6
  16383, // 7
  20000, // 8
  32767, // 9
  30000, // 10
  40000, // 11
  65535  // 12
};

int16_t ROTATION_DEG; // milos
int32_t CPR; // milos
int32_t ROTATION_MAX; // milos
int32_t ROTATION_MID; // milos
uint16_t MM_MIN_MOTOR_TORQUE; // milos, loaded from EEPROM
uint16_t MM_MAX_MOTOR_TORQUE; // milos, loaded from EEPROM
uint16_t MAX_DAC; // milos, loaded from EEPROM
uint16_t TOP; // milos, pwmstate byte loaded from EEPROM, then in InitPWM() function calcTOP(pwmstate) defines TOP value

uint16_t calcTOP(byte b) { // milos, added - function which returns TOP value from pwmstate byte argument
#ifndef USE_MCP4725
  byte j = 0b00000000; // index of frequency and PWM resolution
  for (uint8_t i = 0; i < 4; i++) {
    bitWrite(j, i, bitRead(b, i + 2)); // milos, decode bits2-5 from pwmstate byte into index
  }
  if (bitRead(b, 0)) { // if phase correct PWM mode (pwmstate bit0=1)
    return (PWMtops[j] >> 1); // milos, divide by 2
  } else { // if fast PWM mode (pwmstate bit0=0)
    return (PWMtops[j]);
  }
#else // if mcp4725
  return (MAX_DAC);
#endif
}

typedef struct s32v { // milos, added - 2 dimensional vector structure (for ffb and position)
  s32 x;
#ifdef USE_TWOFFBAXIS // milos, code optimization
  s32 y;
#endif // end of 2 ffb axis
};

f32 FFB_bal; // milos, FFB balance slider
f32 L_bal; // milos, left PWM balance multiplier
f32 R_bal; // milos, right PWM balance multiplier
f32 minTorquePP; // milos, added - min torque in percents

// milos, added - RCM pwm mode definitions
f32 RCM_min = 1.0; // minimal RCM pulse width in ms
f32 RCM_zer = 1.5; // zero RCM pulse width in ms
f32 RCM_max = 2.0; // maximal RCM pulse width in ms

f32 RCMscaler (byte value) { // milos, added - scales correctly RCM pwm mode
  if (bitRead(value, 0)) { // if pwmstate bit0=1
    return 1000.0; // for phase correct pwm mode
  } else {  // if pwmstate bit0=0
    return 2000.0; // for fast pwm mode
  }
}

boolean zIndexFound = false; // milos, added - keeps track if z-index pulse from encoder was found after powerup
#ifdef USE_AS5600
#ifdef USE_CENTERBTN
boolean cButtonPressed = false; // milos, added - gets true when we press center button (activated via interrupt function)
#endif // end of as5600
#endif // end of center btn

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

typedef struct xysh { // milos, added - holds shifter configuration
  uint16_t cal[5]; // calibration limits that define where the gears are
  // i  cal gears (if <=)
  // 0  x0  1
  // 1  x1  3
  // 2  x2  5
  // 3  y0  2,4,6,8, R
  // 4  y1  N
  uint8_t cfg;
  //bit0-revBtn invert,bit1-8 gears,bit2-X invert,bit3-Y invert,bit4-unused,bit5-unused,bit6-unused,bit7-unused
  //bit0=1: reverse gear button inverted (for logitech G25/G27/G29/G923 H-shifters)
  //bit1=1: 8 gear mode (r gear in 8th)
  //bit2=1: X-axis inverted
  //bit3=1: Y-axis inverted
  int16_t x; // horizontal position
  int16_t y; // vertical position
};

// milos - added, function for decoding XY shifter analog values into last 8 buttons
uint32_t decodeXYshifter (uint32_t inbits, xysh *s) {
  uint32_t gears = 0; // shifter gears represented as digital buttons (1 bit for each gear)
  const uint8_t InpRevButtonBit = 0; // input reverse gear button bit number (normal buttons start from bit4, bit0-bit3 are reserved for hat switch)
  const uint8_t OutRevButtonBit = 0; // output reverse gear button bit number (this is where we want to put reverse gear button, out of 24 available buttons)
  if (bitRead(s->cfg, 0)) {   // if reverse gear button is inverted (logitech shifters)
    if (bitRead(inbits, InpRevButtonBit + 4)) { // read rev gear button bit
      bitClear(inbits, InpRevButtonBit + 4); // if 1, set rev gear bit to 0
    } else {
      bitSet(inbits, InpRevButtonBit + 4); // if 0, set rev gear bit to 1
    }
  }
  if (s->x < s->cal[0] && s->y >= s->cal[4]) { // 1st gear
    bitSet(gears, 16);
  } else if (s->x < s->cal[0] && s->y < s->cal[3]) { // 2nd gear
    bitSet(gears, 17);
  } else if (s->x >= s->cal[0] && s->x < s->cal[1] && s->y >= s->cal[4]) { // 3rd gear
    bitSet(gears, 18);
  } else if (s->x >= s->cal[0] && s->x < s->cal[1] && s->y < s->cal[3]) { // 4th gear
    bitSet(gears, 19);
  } else if (s->x >= s->cal[1] && s->x < s->cal[2] && s->y >= s->cal[4]) { // 5th gear
    bitSet(gears, 20);
  } else if (s->x >= s->cal[1] && s->x < s->cal[2] && s->y < s->cal[3]) { // 6th gear
    if (bitRead(s->cfg, 1)) { // if 8 gear shifter
      bitSet(gears, 21); // set 6th gear
    } else { // if 6 gear shifter
      if (bitRead(inbits, InpRevButtonBit + 4)) {
        bitSet(gears, OutRevButtonBit); // reverse gear
      } else {
        bitSet(gears, 21); // still set 6th gear
      }
    }
  } else if (s->x >= s->cal[2] && s->y >= s->cal[4]) { // 7th gear
    bitSet(gears, 22);
  } else if (s->x >= s->cal[2] && s->y < s->cal[3]) { // 8th gear
    if (bitRead(s->cfg, 1)) { // if 8 gear shifter
      if (bitRead(inbits, InpRevButtonBit + 4)) {
        bitSet(gears, OutRevButtonBit); // reverse gear
      } else {
        bitSet(gears, 23); // set 8th gear
      }
    } else { // if 6 gear shifter
      bitSet(gears, 23); // still set 8th gear
    }
  }
  // milos, on leonardo/micro when both xy shifter and hat switch are used pins D5,D6,D7 are remaped to buttons 0,1,2
  // milos, so we need to stop reading these pins twice as normal buttons
  // milos, pins D5,D6,D7 are taken care of by bit mask at bit4,bit5,bit7 in xy shifter decoding function bellow
  uint32_t bitMask = 0b11110000000011111111111111101111; // milos, default bit mask, bit4=0 for reverse gear button, bits20-27 are 0 for gear buttons to be inserted
  //#ifdef USE_XY_SHIFTER // milos, with xy shifter
#ifdef USE_HATSWITCH // milos, with hat switch
#ifndef USE_PROMICRO // milos, only for leonardo/micro
#ifndef USE_SHIFT_REGISTER // milos, for shift register we need default bitmask
  bitMask = 0b11110000000011111111111101001111; // milos, modifyed bit mask not to show duplicate buttons0,1,3
#endif // end of shift register
#else // milos, for proMicro
  bitMask = 0b11110000000011111111111111101111; // milos, default bit mask
#endif // end of proMicro
#else // milos, no hat switch with xy shifter
#ifndef USE_PROMICRO // milos, only for leonardo/micro
  bitMask = 0b11110000000011111111111110001111; // milos, modifyed bit mask not to show duplicate buttons0,1,2
#endif // end of proMicro
#endif // end of hat switch
  //#endif // end of xy shifter

  return ((inbits & bitMask) | (gears << 4)); // milos, gears are shifted to the left by 4 bits to skip updating hat switch, reverse gear is at bit4 (1st bit of buttons)
}

typedef struct s16a { // milos, added - holds individual 16bit axis properties
  int16_t val;
  int16_t min;
  int16_t max;
};

typedef struct s32a { // milos, added - holds individual bit axis properties
  int32_t val; // milos, when using load cell we can have more than 16bit range for brake axis
  int16_t min; // milos, these are used for manual/autocalib so we can keep them 16bit as analog axis are 10bit only
  int16_t max; // milos, when we use load cell min/max are unused for brake axis
};

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

#ifdef USE_TCA9548 // milos, added
#include <Wire.h> // milos, we need it here also
// i2c address of TCA9548A i2c multiplexer chip (each chip has to have appropriate address bits configured)
#define baseTCA0 0x70 // A0,A1,A2 = 000 // base addrress (there are 8)
//          TCA1 0x71 // A0,A1,A2 = 100
//          TCA2 0x72 // A0,A1,A2 = 010
//          TCA3 0x73 // A0,A1,A2 = 110
//          TCA4 0x74 // A0,A1,A2 = 001
//          TCA5 0x75 // A0,A1,A2 = 101
//          TCA6 0x76 // A0,A1,A2 = 011
//          TCA7 0x77 // A0,A1,A2 = 111
// send to i2C info about which TCA chip and which i2C channel to select
void TcaChannelSel(uint8_t addr, uint8_t ch) {
  Wire.beginTransmission(addr);
  Wire.write(1 << ch);
  Wire.endTransmission();
}
#endif // end of tca

#endif // _CONFIG_H_
