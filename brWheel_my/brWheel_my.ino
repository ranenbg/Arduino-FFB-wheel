/* Arduino Leonardo Force Feedback Wheel firmware

  Copyright 2015  Etienne Saint-Paul  (esaintpaul [at] gameseed [dot] fr)
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
#include "ConfigHID.h"
#include "debug.h"
#include "ffb_pro.h"
//#include "ffb.h" // milos, commented out
#include "USBDesc.h"
#ifdef USE_QUADRATURE_ENCODER
#include "QuadEncoder.h"
#endif
#ifdef USE_VNH5019
#include "DualVNH5019MotorShield.h"
#endif
#include <Wire.h>
#ifdef USE_EEPROM
#include <EEPROM.h> // milos, uncommented
#endif
#ifdef USE_LOAD_CELL
#include <HX711_ADC.h> // milos, added
#endif
#ifdef USE_LCD
#include <LiquidCrystal_I2C.h> // milos, added
#endif
#ifdef USE_ADS1015
#include <Adafruit_ADS1015.h> // milos, added
#endif
#ifdef USE_MCP4725
#include <Adafruit_MCP4725.h> // milos, added
#endif
#ifdef USE_AS5600 // milos, added
#include "AS5600.h"
#endif

//extern u8 valueglobal; // milos, commented out

//--------------------------------------- Globals --------------------------------------------------------

//b8 fault; // milos, commented out
fwOpt fwOptions; // milos, added - struct that holds all firmware options
s16a accel, clutch, hbrake; // milos, changed from s16
#ifdef USE_XY_SHIFTER
xysh shifter; // milos, added
#endif // end of xy shifter
s32a brake; // milos, we need 32bit due to 24 bits on load cell ADC, changed from s32
s32 turn; // milos, x-axis (for optical or magnetic encoder)
s32v axis; // milos, struct containing x and y-axis position input for calculating xy ffb
u32 button = 0; // milos, added

//milos, added
#ifdef USE_ADS1015
//Adafruit_ADS1115 ads(0x48);     /* Use this for the 16-bit version */
Adafruit_ADS1015 ads(0x48);    /* Use this for the 12-bit version */
#endif

#ifdef USE_MCP4725  //milos, added
Adafruit_MCP4725 dac0; // address 0x60, address pin connected to GND (or dissconected), left Force channel
Adafruit_MCP4725 dac1; // address 0x61, address pin connected to VCC,                   right Force channel
#endif

cFFB gFFB;
BRFFB brWheelFFB;

//milos, added
#ifdef AVG_INPUTS
extern s32 analog_inputs[];
u8 asc = 0; // milos, added - sample counter for averaging of analog inputs
#endif

u32 last_ConfigSerial = 0;
u32 last_refresh = 0;
s32v ffbs; // milos, instance of struct holding 2 axis FFB data
u32 now_micros = micros();
u32 timeDiffConfigSerial = now_micros;

uint16_t dz, bdz; // milos
uint8_t last_LC_scaling; //milos
//----------------------------------------- Options -------------------------------------------------------

#ifdef USE_LOAD_CELL // milos, added
//HX711 constructor (dt pin, sck pin)
HX711_ADC LoadCell(4, 5); // milos, added
#endif

#ifdef USE_QUADRATURE_ENCODER
cQuadEncoder myEnc;
#endif

#ifdef USE_LCD  // milos, added
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD i2C address
#endif

#ifdef USE_VNH5019
DualVNH5019MotorShield ms;
void stopIfFault() {
  if (ms.getM1Fault()) {
    DEBUG_SERIAL.println("M1 fault");
    while (1);
  }
  if (ms.getM2Fault()) {
    DEBUG_SERIAL.println("M2 fault");
    while (1);
  }
}
#endif

#ifdef USE_AS5600 // milos, added
AS5600L as5600(0x36); // uses default wire.h, milos added fixed i2C address of AS5600
#endif

//--------------------------------------------------------------------------------------------------------
//-------------------------------------------- SETUP -----------------------------------------------------
//--------------------------------------------------------------------------------------------------------

void setup() {
  CONFIG_SERIAL.begin(115200);
  //fault = false; // milos, commented out
  accel.val = 0;
  brake.val = 0;
  clutch.val = 0;
  turn = 0;
  axis.x = 0;
#ifdef USE_TWOFFBAXIS
  axis.y = 0;
#endif // end of 2 ffb axis

  //pinModeFast(LCSYNC_LED_PIN,OUTPUT);
  //pinModeFast(SYNC_LED_PIN, OUTPUT);
  //pinModeFast(LED_PIN, OUTPUT);

  //pinMode(SCK,INPUT); //11,INPUT);
  //pinMode(MISO,INPUT); //12,INPUT);
  //pinMode(MOSI,INPUT); //13,INPUT);

#ifdef USE_EEPROM
  SetEEPROMConfig(); // milos, check firmware version from EEPROM (if any) and load defaults if required
  LoadEEPROMConfig(); // milos, read firmware setings from EEPROM and update current firmware settings
#else //milos, otherwise you can set it here (firmware settings will be set to these defaults on each arduino power up or reset)
  // CPR (counts per revolution) depends on encoder's pulse per revolution - PPR, and gear ratio - GR you have coupling it with wheel shaft
  // mine has PPR=600 and I use 10:1 GR (for 1 wheel rotation I have 10 rotaions of encoder's shaft)
  // each pulse has 4 states (counts), so final equation for CPR is:
  // CPR = 600*10*4 = 24000
  ROTATION_DEG = 1080; // milos, here we set wheel's initial deg of rotation (can be changed latter through serial interface, or my program wheel_control.pde written in processing)
  CPR = 2400; // milos, here we set our optical encoder's counts per (wheel shaft) revolution (this is a constant value - your hardware/mechanical parameter)
  configGeneralGain = 100;
  configDamperGain = 50;
  configFrictionGain = 50;
  configConstantGain = 100;
  configPeriodicGain = 100;
  configSpringGain = 50;
  configInertiaGain = 50;
  configCenterGain = 70;
  configStopGain = 100;
  effstate = 0b00000001; // milos, only desktop spring effect is enabled
#ifdef USE_LOAD_CELL
  LC_scaling = 45; // milos, brake pressure for load cell (arbitrary units)
#else
  LC_scaling = 128; // milos, FFB left/right balance (128=center)
#endif
#ifndef USE_MCP4725
  //pwmstate = 0b00001001; // milos, PWM out enabled, phase correct, pwm+-, 16kHz, TOP 500
  pwmstate = 0b00001100; // milos, PWM out enabled, fast pwm, pwm+-, 7.8kHz, TOP 11bit (2047)
#else
  pwmstate = 0b10000000; // milos, DAC out enabled, DAC+- mode
#endif // end of mcp4725
  MM_MIN_MOTOR_TORQUE = 0;
  minTorquePP = 0.0;
#ifdef USE_AUTOCALIB //milos, reset limits for autocalibration of pedals
  accel.min = Z_AXIS_LOG_MAX;
  accel.max = 0;
  brake.min = s32(Z_AXIS_LOG_MAX);
  brake.max = 0;
  clutch.min = RX_AXIS_LOG_MAX;
  clutch.max = 0;
  hbrake.min = RY_AXIS_LOG_MAX;
  hbrake.max = 0;
#else
  accel.min = 0;
  accel.max = maxCal;
  brake.min = 0;
  brake.max = maxCal;
  clutch.min = 0;
  clutch.max = maxCal;
  hbrake.min = 0;
  hbrake.max = maxCal;
#endif // end of autocalib
#ifdef USE_XY_SHIFTER // milos, set default shifter config
  shifter.cal[0] = 255;
  shifter.cal[1] = 511;
  shifter.cal[2] = 767;
  shifter.cal[3] = 255;
  shifter.cal[4] = 511;
  shifter.cfg = 0;
#endif // end of xy shifter
#endif // end of eeprom
  ROTATION_MAX = int32_t(float(CPR) / 360.0 * float(ROTATION_DEG)); // milos
  ROTATION_MID = ROTATION_MAX >> 1; // milos

#ifdef USE_QUADRATURE_ENCODER
#ifndef USE_AS5600
  //myEnc.Init(ROTATION_MID + brWheelFFB.offset, true); //ROTATION_MID + gCalibrator.mOffset); // milos, pullups enabled
  myEnc.Init(ROTATION_MID, true); // milos, pullups enabled, do not apply any encoder offset at this point
#endif // end of as5600
#endif // end of quad enc

#ifdef USE_CONFIGHID // milos, used only through HID configuration interface
  update(&fwOptions); // milos, added - update firmware options based on Config.h predefines
#endif // end of config hid
  InitInputs();
  FfbSetDriver(0);

#ifdef USE_VNH5019
  ms.init();
#endif

  InitPWM(); // milos, initialize PWM (or DAC) settings
  ffbs.x = 0; // milos, init xFFB at zero
#ifdef USE_TWOFFBAXIS
  ffbs.y = 0; // milos init yFFB at zero
#endif // end of 2 ffb axis
  SetPWM(&ffbs); // milos, zero PWM at startup

#ifdef USE_QUADRATURE_ENCODER
#ifndef USE_AS5600
  (CALIBRATE_AT_INIT ? brWheelFFB.calibrate() : myEnc.Write(ROTATION_MID)); // milos, allways set encoder at 0deg (ROTATION_MID) at startup and calibrate if enabled
#endif // end of as5600
#endif // end of quad enc

#ifdef USE_LCD // milos, not fully implemented yet
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("fw-v");
  lcd.print(VERSION, DEC);
#endif

#ifdef USE_AUTOCALIB
#ifdef AVG_INPUTS
  dz = 8; // milos, set the accel, brake and clutch pedal dead zones that will be taken from min and max axis val (default is 8 out of 4095)
#else //
  dz = 2; // milos, set 2 out of 1023
#endif // end of avg inputs
#else // when no autocal
  dz = 0; // milos, we can set min/max cal limits manualy so we do not need this
#endif // end of autocal
  bdz = 2047; // milos, set the brake pedal dead zone taken from min axis val, when using load cell (default is 2047 out of 65535)
  last_LC_scaling = LC_scaling; // milos, update last load cell scaling value (brake pressure)

#ifdef USE_ADS1015 // milos, added
  // When using ADS1015 board, all inputs are 12 bits resolution (4096 total steps, 0-4095 range)
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  ads.begin(); //milos, added
#endif

#ifdef USE_AS5600
#ifndef USE_ADS1015
  Wire.begin(); // milos, D2-SDA and D3-SCL by default on Leonardo, Micro and proMicro boards
#endif
  as5600.begin();
  as5600.setFastFilter(0); // milos, configure fast filter threshold (0-OFF is slow filter enable)
  as5600.setSlowFilter(0); // milos, configure slow filter or readout precision: 0-best(slowest), 3-worst(fastest)
  //as5600.setAddress(0x36); // milos, not needed here, i2C address already defined in function constructor
  //as5600.setDirection(AS5600_CLOCK_WISE); // milos, not needed, but DIR pin should be on GND (not sure if it has a pulldown resistor)
  as5600.resetCumulativePosition(ROTATION_MID); // milos, initialize at 0deg at startup
#endif
  last_refresh = micros();
}

//--------------------------------------------------------------------------------------------------------
//------------------------------------ Main firmware loop ------------------------------------------------
//--------------------------------------------------------------------------------------------------------

void loop() {
#ifdef AVG_INPUTS //milos, added option see config.h
  if (asc < avgSamples) {
    ReadAnalogInputs(); // milos, get readings for averaging (only do it until we get all samples)
    asc++; // milos
  }
#endif // end of avg inp

  now_micros = micros(); // milos, we are polling the loop (FFB and USB reports are sent periodicaly)
  {
    timeDiffConfigSerial = now_micros - last_ConfigSerial; // milos, timer for serial interface

    if ((now_micros - last_refresh) >= CONTROL_PERIOD) {
#ifdef AVG_INPUTS //milos
      asc = 0; // milos, reset counter for averaging
#endif // end of avg_inputs
      last_refresh = now_micros;  // milos, timer for FFB and USB reports
      //SYNC_LED_HIGH(); // milos
#ifdef  USE_SHIFT_REGISTER
      for (uint8_t i = 0; i <= SHIFTS_NUM; i++) { // milos, read all states in one pass
        nextInputState();  // milos, refresh state of shift register and read the next incoming bit
      }
#endif // end of shift register
#ifndef USE_AS5600 // milos, if AS5600 is not enabled quadrature encoder is used
#ifdef USE_QUADRATURE_ENCODER
      if (zIndexFound) {
        turn = myEnc.Read() - ROTATION_MID + brWheelFFB.offset; // milos, only apply z-index offset if z-index pulse is found
      } else {
        turn = myEnc.Read() - ROTATION_MID;
      }
#else // milos, if no optical enc and no as5600, use pot for X-axis
      turn = map(accel.val, 0, Z_AXIS_PHYS_MAX, -ROTATION_MID - 1, ROTATION_MID); // milos, X-axis on accelerator
#endif // end of quad enc
#else // if we use as5600
#ifdef USE_CENTERBTN
      if (cButtonPressed) {
        as5600.resetCumulativePosition(ROTATION_MID);  // when using magnetic encoder set X-axis to 0deg
        cButtonPressed = false;
      }
#endif // end of center btn
      turn = as5600.getCumulativePosition() - ROTATION_MID; // milos, AS5600 angle readout
#endif // end of as5600
      axis.x = turn; // milos, xFFB on X-axis (optical or magnetic encoder)
#ifdef USE_TWOFFBAXIS // milos, if 2 ffb axis, use Y-axis as input for yFFB axis
      axis.y = map(brake.val, 0, Y_AXIS_PHYS_MAX, -ROTATION_MID - 1, ROTATION_MID); // milos, temporary Y axis for yFFB force, scaled according to the encoder step size
#endif // end of 2 ffb axis

#ifdef USE_ANALOGFFBAXIS
      if (indxFFBAxis(effstate) == 1) {
        axis.x = map(brake.val, 0, Y_AXIS_PHYS_MAX, -ROTATION_MID - 1, ROTATION_MID); // milos, xFFB on Y-axis
      } else if (indxFFBAxis(effstate) == 2) {
        axis.x = map(accel.val, 0, Z_AXIS_PHYS_MAX, -ROTATION_MID - 1, ROTATION_MID); // milos, xFFB on Z-axis
      } else if (indxFFBAxis(effstate) == 3) {
        axis.x = map(clutch.val, 0, RX_AXIS_PHYS_MAX, -ROTATION_MID - 1, ROTATION_MID); // milos, xFFB on RX-axis
      } else if (indxFFBAxis(effstate) == 4) {
        axis.x = map(hbrake.val, 0, RY_AXIS_PHYS_MAX, -ROTATION_MID - 1, ROTATION_MID);  // milos, xFFB on RY-axis
      }
#endif // end of analog ffb axis
      ffbs = gFFB.CalcTorqueCommands(&axis); // milos, passing pointer struct with x and y-axis, in encoder raw units -inf,0,inf
      turn *= f32(X_AXIS_PHYS_MAX) / f32(ROTATION_MAX); // milos, conversion to physical HID units
      turn = constrain(turn, -MID_REPORT_X - 1, MID_REPORT_X); // milos, -32768,0,32767 constrained to signed 16bit range

      SetPWM(&ffbs); // milos, FFB signal is generated as digital PWM or analog DAC output (ffbs is a struct containing 2-axis FFB, here we pass it as pointer for calculating PWM or DAC signals)
      //SYNC_LED_LOW(); //milos
      // USB Report
      {
        //last_send = now_micros;
#ifdef AVG_INPUTS //milos, added option see config.h
        AverageAnalogInputs();				// Average readings
#endif

#ifdef USE_ADS1015 // milos, if you plan to use ADS1015 for all 3 pedals (no load cell) then use readADC_SingleEnded, for full 12bit use differential reading but only 2 such diff inputs are available
        accel.val = constrain(ads.readADC_SingleEnded(ACCEL_INPUT), 0 , 2047); //milos, Z axis, 11bit
        clutch.val = constrain(ads.readADC_SingleEnded(CLUTCH_INPUT), 0 , 2047); //milos, RX axis, 11bit
        hbrake.val = constrain(ads.readADC_SingleEnded(HBRAKE_INPUT), 0 , 2047); //milos, RY axis, 11bit
        //accel.val = constrain(ads.readADC_Differential_0_1()+2048, 0, 4095);  //milos, Z axis, 12bit
        //clutch.val = constrain(ads.readADC_Differential_2_3()+2048, 0, 4095); //milos, RX axis, 12bit

#else //if no ads1015
#ifdef AVG_INPUTS // milos, we do not average h-shifter axis, only pedal axis
        accel.val = analog_inputs[ACCEL_INPUT];
#ifdef USE_PROMICRO
#ifndef USE_XY_SHIFTER
        clutch.val = analog_inputs[CLUTCH_INPUT];
        hbrake.val = analog_inputs[HBRAKE_INPUT];
#else // milos, if we use h-shifter on proMicro with avg inputs
        clutch.val = 0;
        hbrake.val = 0;
        shifter.x = analogRead(CLUTCH_PIN); // milos
        shifter.y = analogRead(HBRAKE_PIN); // milos
#endif // end of xy shifter
#else // for leonardo we can avg pedal inputs and also have h-shifter axis
        clutch.val = analog_inputs[CLUTCH_INPUT];
        hbrake.val = analog_inputs[HBRAKE_INPUT];
#ifdef USE_XY_SHIFTER
        shifter.x = analogRead(SHIFTER_X_PIN); // milos
        shifter.y = analogRead(SHIFTER_Y_PIN); // milos
#endif // end of h-shifter
#endif // end of proMicro
#else // if no avg
        accel.val = analogRead(ACCEL_PIN); // milos, Z axis
#ifndef USE_PROMICRO // milos, for Leonardo and Micro
#ifndef USE_EXTRABTN // milos, we can have clutch and hbrake only when not using extra buttons
        clutch.val = analogRead(CLUTCH_PIN); // milos, RX axis
        hbrake.val = analogRead(HBRAKE_PIN); // milos, RY axis
#else // if extra buttons
        clutch.val = 0; // milos, RX axis
        hbrake.val = 0; // milos, RY axis
#endif // end of extra button
#ifdef USE_XY_SHIFTER // milos
        shifter.x = analogRead(SHIFTER_X_PIN); // milos
        shifter.y = analogRead(SHIFTER_Y_PIN); // milos
#endif // end of xy shifter
#else // if we use proMicro
#ifdef USE_XY_SHIFTER // milos, compromize - for proMicro with XY shifter, we can't have clutch and handbrake
        clutch.val = 0; // milos, RX axis
        hbrake.val = 0; // milos, RY axis
        shifter.x = analogRead(CLUTCH_PIN); // milos, use clutch analog input instead
        shifter.y = analogRead(HBRAKE_PIN); // milos, use handbrake analog input instead
#else // for proMicro, when no XY shifter
#ifndef USE_EXTRABTN // milos, only available if not using extra buttons
        clutch.val = analogRead(CLUTCH_PIN); // milos, RX axis
        hbrake.val = analogRead(HBRAKE_PIN); // milos, RY axis
#else // if using extra buttons
#ifndef USE_LOAD_CELL
        clutch.val = 0; // milos, RX axis unavailable when no lc
#else // if no load cell
        clutch.val = analogRead(CLUTCH_PIN); // milos, RX axis is available if we use lc
#endif // end of use lc
        hbrake.val = 0; // milos, RY axis is allways unavailable
#endif // end of extra button
#endif // end of xy shifter
#endif // end proMicro
#endif // end of avg
#endif // end of ads

#ifdef USE_LOAD_CELL // milos, when use LC
        if (LC_scaling != last_LC_scaling) { // milos, apply only if changed (through serial interface)
          LoadCell.setCalFactor(0.25 * float(LC_scaling)); // user set calibration factor (float)
          last_LC_scaling = LC_scaling; // milos, update new value
          LoadCell.tare(); // milos, zero out the measurement using new scaling factor
        }
        // milos, update() should be called at least as often as HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS
        // milos, we are calling it in every polling interval
        LoadCell.update(); // milos, I have configured mine for 80Hz reading by applying 5V at pin15 of HX711 chip (by default it's only 10Hz because pin15 is grounded in PCB, you must cut this trace)
        brake.val = LoadCell.getData(); // milos, read smoothed data from LC (running average of 1 sample, see provided HX711_ADC.h and adjust there if more samples are needed)
#else // milos, when no LC
#ifdef USE_ADS1015
        brake.val = constrain(ads.readADC_SingleEnded(BRAKE_INPUT), 0 , 2047); // milos, Y axis, 11bit
#else // if no ads
#ifdef AVG_INPUTS // milos, added option
        brake.val = analog_inputs[BRAKE_INPUT];
#else // if no avg
        brake.val = analogRead(BRAKE_PIN); // milos, Y axis
#endif // end of avg
#endif // end of ads
#endif // end of lc

#ifdef  USE_AUTOCALIB // milos, update limits for pedal autocalibration
        if (accel.val < accel.min) accel.min = accel.val;
        if (accel.val > accel.max) accel.max = accel.val;
        if (brake.val < brake.min) brake.min = brake.val;
        if (brake.val > brake.max) brake.max = brake.val;
        if (clutch.val < clutch.min) clutch.min = clutch.val;
        if (clutch.val > clutch.max) clutch.max = clutch.val;
        if (hbrake.val < hbrake.min) hbrake.min = hbrake.val;
        if (hbrake.val > hbrake.max) hbrake.max = hbrake.val;
#endif // end of autocalib
#ifdef USE_AVGINPUTS
        // milos, update calibration limits for increased axis resolution due to averaging (depends on num of samples)
        accel.min *= avgSamples;
        accel.max *= avgSamples;
#ifndef USE_LOADCELL
        brake.min *= avgSamples;
        brake.max *= avgSamples;
#endif // end of load cell
        clutch.min *= avgSamples;
        clutch.max *= avgSamples;
        hbrake.min *= avgSamples;
        hbrake.max *= avgSamples;
#endif // end of avg inputs

        // milos, rescale all analog axis according to a new manual calibration and add small deadzones
        accel.val = map(accel.val, accel.min + dz, accel.max - dz, 0, Z_AXIS_PHYS_MAX);  // milos, with manual calibration and dead zone
        clutch.val = map(clutch.val, clutch.min + dz, clutch.max - dz, 0, RX_AXIS_PHYS_MAX);
        hbrake.val = map(hbrake.val, hbrake.min + dz, hbrake.max - dz, 0, RY_AXIS_PHYS_MAX);
        accel.val = constrain(accel.val, 0, Z_AXIS_PHYS_MAX); // milos, constrain axis ranges
        clutch.val = constrain(clutch.val, 0, RX_AXIS_PHYS_MAX);
        hbrake.val = constrain(hbrake.val, 0, RY_AXIS_PHYS_MAX);

#ifdef USE_LOAD_CELL // milos, with load cell
        if (brake.val < bdz) { // milos, if values below deadzone threshold
          brake.val = 0; // milos, truncate
        } else {
          brake.val = map(brake.val, bdz, Y_AXIS_PHYS_MAX + bdz, 0, Y_AXIS_PHYS_MAX); // milos, no autocalibration
        }
#else // milos, when no load cell
        brake.val = map(brake.val, brake.min + dz, brake.max - dz, 0, Y_AXIS_PHYS_MAX); // milos, for both manual and auto cal
#endif // end of load cell
        brake.val = constrain(brake.val, 0, Y_AXIS_PHYS_MAX); // milos

        button = readInputButtons(); // milos, read all buttons including matrix and hat switch

#ifdef USE_XY_SHIFTER // milos, added
        if ((bitRead(shifter.cfg, 2))) shifter.x = 1023 - shifter.x; // milos, invert shifter X-axis
        if ((bitRead(shifter.cfg, 3))) shifter.y = 1023 - shifter.y; // milos, invert shifter Y-axis
        button = decodeXYshifter(button, &shifter); // milos, added - convert analog XY shifter values into last 8 buttons
#endif //end of xy shifter

#ifdef USE_QUADRATURE_ENCODER // milos, if we have quad enc
        SendInputReport(turn + MID_REPORT_X + 1, brake.val, accel.val, clutch.val, hbrake.val, button); // milos, X, Y, Z, RX, RY, hat+button; (0-65535) X-axis range, center at 32768
#else // milos, if no quad enc
#ifdef USE_AS5600 // milos, if we have as5600
        SendInputReport(turn + MID_REPORT_X + 1, brake.val, accel.val, clutch.val, hbrake.val, button); // milos
#else // milos, if no quad enc and no as5600, Z-axis (accel) is used for X-axis, but we have have to send something instead of Z-axis -> half axis value for example
        SendInputReport(turn + MID_REPORT_X + 1, brake.val, Z_AXIS_PHYS_MAX >> 1, clutch.val, hbrake.val, button); // milos
#endif // end of as5600
#endif // end of quad enc

#ifdef AVG_INPUTS //milos, added option see config.h
        ClearAnalogInputs();
#endif // end of avg inp
#ifdef USE_CONFIGCDC
        if (timeDiffConfigSerial >= CONFIG_SERIAL_PERIOD) {
          configCDC(); // milos, configure firmware with virtual serial port
          last_ConfigSerial = now_micros;
        }
#endif // end of use config cdc
      }
      //SYNC_LED_LOW(); //milos
    }
  }
}
