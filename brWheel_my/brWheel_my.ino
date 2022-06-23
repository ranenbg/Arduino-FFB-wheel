
/* Force Feedback Wheel

  Copyright 2015  Etienne Saint-Paul  (esaintpaul [at] gameseed [dot] fr)
  Copyright 2017  Fernando Igor  (fernandoigor [at] msn [dot] com)
  Copyright 2018-2022  Milos Rankovic (ranenbg [at] gmail [dot] com)

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

#include "ffb.h"
#include "ffb_pro.h"
#ifdef USE_VNH5019
#include "DualVNH5019MotorShield.h"
#endif
#include "debug.h"
#include <Wire.h>
#include <digitalWriteFast.h>
#ifdef USE_EEPROM
#include <EEPROM.h> //milos, uncommented
#endif
#include "Config.h"
#include "QuadEncoder.h"
#ifdef USE_LOAD_CELL
#include <HX711_ADC.h> //milos, added
#endif
#ifdef USE_LCD
#include <LiquidCrystal_I2C.h> //milos, added
#endif
#ifdef USE_ADS1105
#include <Adafruit_ADS1015.h> //milos, added
#endif
#include "USBDesc.h"
#include "ConfigHID.h"
#ifdef USE_MCP4725
#include <Adafruit_MCP4725.h> //milos, added
#endif

extern u8 valueglobal;

//--------------------------------------- Globals --------------------------------------------------------

/*u8 spi_buffer[4];
  u8 spi_bp = 0;
  u8 spi_data_ready = false;*/

b8 fault;
s16 accel, clutch, hbrake;
s32 brake; //milos, we need 32bit due to 24 bits on LC ADC
s32 turn;
u16 accelMin = Z_AXIS_LOG_MAX, accelMax = 0; // milos
s16 brakeMin = s16(Z_AXIS_LOG_MAX), brakeMax = 0; // milos, must be signed
u16 clutchMin = RX_AXIS_LOG_MAX, clutchMax = 0; // milos
u16 hbrakeMin = RY_AXIS_LOG_MAX, hbrakeMax = 0; // milos
u32 button = 0; //milos, added

//milos, added
#ifdef USE_ADS1105
//Adafruit_ADS1115 ads(0x48);     /* Use this for the 16-bit version */
Adafruit_ADS1015 ads(0x48);    /* Use this for the 12-bit version */
#endif

#ifdef USE_MCP4725  //milos, added
Adafruit_MCP4725 dac0; // address 0x60, address pin connected to GND (or dissconected), left Force channel
Adafruit_MCP4725 dac1; // address 0x61, address pin connected to VCC,                   right Force channel
#endif

//s32 shifterX, shifterY; // milos, commented

cFFB gFFB;
BRFFB brWheelFFB;

//milos, added
#ifdef AVG_INPUTS
extern s32 analog_inputs[];
#endif

//u32 last_send = 0;
//u32 last_nextShift = 0;
u32 last_ConfigSerial = 0;
u32 last_refresh = 0;
s32 command = 0;
u32 now_micros = micros();
//u32 time_diff = now_micros;
//u32 timeDiffNextShift = now_micros;
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
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#endif

#ifdef USE_VNH5019
DualVNH5019MotorShield ms;
void stopIfFault()
{
  if (ms.getM1Fault())
  {
    DEBUG_SERIAL.println("M1 fault");
    while (1);
  }
  if (ms.getM2Fault())
  {
    DEBUG_SERIAL.println("M2 fault");
    while (1);
  }
}
#endif
//--------------------------------------------------------------------------------------------------------
//-------------------------------------------- SETUP -----------------------------------------------------
//--------------------------------------------------------------------------------------------------------

void setup() {
  CONFIG_SERIAL.begin(115200);

  //last_nextShift = micros();
  //last_send = micros();
  last_refresh = micros();
  fault = false;
  accel = 0;
  brake = 0;
  clutch = 0;
  turn = 0;

  //pinModeFast(LCSYNC_LED_PIN,OUTPUT);
  //pinModeFast(SYNC_LED_PIN, OUTPUT);
  //pinModeFast(LED_PIN, OUTPUT);

  //pinMode(SCK,INPUT); //11,INPUT);
  //pinMode(MISO,INPUT); //12,INPUT);
  //pinMode(MOSI,INPUT); //13,INPUT);

#ifdef USE_EEPROM
  SetEEPROMConfig(); //milos, added, checks version and loads defaults
  LoadEEPROMConfig(); //milos, read last values from EEPROM and update all parameters
#else //milos, otherwise you can set it here (ffb settings will be set to these defaults on each power up or reset)
  // CPR (counts per revolution) depends on encoder's pulse per revolution - PPR, and gear ratio - GR you have coupling it with wheel shaft
  // mine has PPR=600 and I use 10:1 GR (for 1 wheel rotation I have 10 rotaions of encoder's shaft)
  // each pulse has 4 states (counts), so final equation for CPR is:
  // CPR = 600*10*4 = 24000
  ROTATION_DEG = 1080; // milos, here we set wheel's initial deg of rotation (can be changed latter through serial interface, or my program wheelControl.pde written in processing)
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
  effstate = 0b00000001; //only desktop spring effect is enabled
#ifdef USE_LOAD_CELL
  LC_scaling = 45; // milos, brake pressure
#else
  LC_scaling = 128; // milos, FFB balance
#endif
#ifndef USE_MCP4725
  pwmstate = 0b00001001; // milos, PWM out enabled, phase correct, pwm+-, 16kHz, TOP 500
#else
  pwmstate = 0b10000000; //milos, DAC out enabled, DAC+- mode
#endif
  MM_MIN_MOTOR_TORQUE = 0;
  minTorquePP = 0;
#endif
  ROTATION_MAX = int32_t(float(CPR) / 360.0 * float(ROTATION_DEG)); // milos
  ROTATION_MID = ROTATION_MAX / 2; // milos

  //myEnc.Init(ROTATION_MID + brWheelFFB.offset, true); //ROTATION_MID + gCalibrator.mOffset); // milos, pullups enabled
  myEnc.Init(ROTATION_MID, true); // milos, pullups enabled, do not apply any encoder offset here

  InitInputs();
  FfbSetDriver(0);

#ifdef USE_VNH5019
  ms.init();
#endif

  InitPWM(); // milos, initialize PWM (or DAC) settings
  SetPWM(0); // milos, set PWM (or DAC) to 0 at startup

#ifdef USE_QUADRATURE_ENCODER
  (CALIBRATE_AT_INIT ? brWheelFFB.calibrate() : myEnc.Write(ROTATION_MID)); //milos, allways set encoder at 0deg (ROTATION_MID) at startup
#endif

#ifdef USE_LCD //milos
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("fw-v");
  lcd.print(VERSION, DEC);
#endif

  dz = 8; // milos, set the accel, brake and clutch pedal dead zones that will be taken from min and max axis val (default is 8 out of 4095)
  bdz = 2047; // milos, set the brake pedal dead zone taken from min axis val, when using load cell (default is 2047 out of 65535)
  last_LC_scaling = LC_scaling; // milos

#ifdef USE_ADS1105 // milos, added
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
}

//--------------------------------------------------------------------------------------------------------
void loop() {
#ifdef AVG_INPUTS //milos, added option see config.h
  ReadAnalogInputs();        // Some reading to take an average
#endif

  now_micros = micros();
  {
    //time_diff = now_micros - last_send;
    //timeDiffNextShift = now_micros - last_nextShift;
    timeDiffConfigSerial = now_micros - last_ConfigSerial;

#ifdef USE_QUADRATURE_ENCODER
    if ((now_micros - last_refresh) >= CONTROL_PERIOD) {
      last_refresh = now_micros;
      //SYNC_LED_HIGH(); // milos
#ifdef  USE_SHIFT_REGISTER
      for (uint8_t i = 0; i <= SHIFTS_NUM; i++) { // milos, added (read all states in one pass)
        nextInputState();           // milos, refresh state of shift-register and read incoming bit
      }
#endif
      if (zIndexFound) {
        turn = myEnc.Read() - ROTATION_MID + brWheelFFB.offset; //milos, only apply z-index offset if z-index pulse is found
      } else {
        turn = myEnc.Read() - ROTATION_MID;
      }

      command = gFFB.CalcTorqueCommand(turn);
      turn *= f32(X_AXIS_PHYS_MAX) / f32(ROTATION_MAX); //milos
      turn = constrain(turn, -MID_REPORT_X, MID_REPORT_X); //milos

      SetPWM(command); // milos, FFB signal generated as PWM (or DAC) output
      //SYNC_LED_LOW(); //milos
      // USB Report
      {
        //last_send = now_micros;
#ifdef AVG_INPUTS //milos, added option see config.h
        AverageAnalogInputs();				// Average readings
#endif
        /*accel = analog_inputs[ACCEL_INPUT]; // milos, commented
          brake = analog_inputs[BRAKE_INPUT];
          clutch = analog_inputs[CLUTCH_INPUT];
          shifterX = analog_inputs[SHIFTER_X_INPUT];
          shifterY = analog_inputs[SHIFTER_Y_INPUT];*/

        /*ads0 = ads.readADC_Differential_0_1();  // milos, diff input between A0 and A1
          ads1 = ads.readADC_Differential_2_3();  // milos, diff input between A2 and A3
          ads2 = ads.readADC_SingleEnded(0);  // milos, single input A0*/

#ifdef USE_ADS1105 // milos, if you plan to use ADS1105 for all 3 pedals (no load cell), then change to readADC_SingleEnded
        accel = constrain(ads.readADC_SingleEnded(ACCEL_INPUT) * 2, 0 , 4095); //milos, Z axis, 11bit
        clutch = constrain(ads.readADC_SingleEnded(CLUTCH_INPUT) * 2, 0 , 4095); //milos, RX axis, 11bit
        hbrake = constrain(ads.readADC_SingleEnded(HBRAKE_INPUT) * 2, 0 , 4095); //milos, RX axis, 11bit
        //accel = constrain(ads.readADC_Differential_0_1() + 2047, 0, 4095);  //milos, Z axis, 12bit
        //clutch = constrain(ads.readADC_SingleEnded(2) * 2, 0 , 4095); //milos, RX axis, 11bit

#else //if no ads
#ifdef AVG_INPUTS //milos, added option
        accel = analog_inputs[ACCEL_INPUT];
        clutch = analog_inputs[CLUTCH_INPUT];
        hbrake = analog_inputs[HBRAKE_INPUT];
#else //if no avg
        accel = analogRead(ACCEL_PIN) * 4; // milos, Z axis
        clutch = analogRead(CLUTCH_PIN) * 4; // milos, RX axis
        hbrake = analogRead(HBRAKE_PIN) * 4; // milos, RY axis
#endif //end of avg
#endif //end of ads

#ifdef USE_LOAD_CELL // milos, when use LC
        if (LC_scaling != last_LC_scaling) {
          LoadCell.setCalFactor(0.25 * float(LC_scaling)); // user set calibration factor (float) // milos, apply only if changed (through serial interface)
          last_LC_scaling = LC_scaling;
          LoadCell.tare(); //milos, zero out the measurement
        }
        //update() should be called at least as often as HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS
        //longer delay in sketch will reduce effective sample rate (be carefull with delay() in loop)
        LoadCell.update(); //milos, I have configured mine for 80Hz reading by applying 5V at pin15 of HX711 chip (by default it's 10Hz, pin15 grounded)
        brake = LoadCell.getData(); // milos, read smoothed data from LC (running average of 1 samples, see HX711_ADC.h I modded it)
#else // milos, when no LC
#ifdef USE_ADS1105
        brake = constrain(ads.readADC_SingleEnded(BRAKE_INPUT) * 2, 0 , 4095); //milos, Y axis, 11bit
#else //if no ads
#ifdef AVG_INPUTS //milos, added option
        brake = analog_inputs[BRAKE_INPUT];
#else //if no avg
        brake = analogRead(BRAKE_PIN) * 4; // milos, Y axis
#endif //end of avg
#endif //end of ads
#endif //end of lc

        // milos, update limits for autocalibration
        if (accel < accelMin) accelMin = accel;
        if (accel > accelMax) accelMax = accel;
        if (brake < brakeMin) brakeMin = brake;
        if (brake > brakeMax) brakeMax = brake;
        if (clutch < clutchMin) clutchMin = clutch;
        if (clutch > clutchMax) clutchMax = clutch;
        if (hbrake < hbrakeMin) hbrakeMin = hbrake;
        if (hbrake > hbrakeMax) hbrakeMax = hbrake;

        // milos, rescale all axis according to new calibration
#ifdef  USE_AUTOCALIB
        accel = map(accel, accelMin + dz, accelMax - dz, 0, Z_AXIS_PHYS_MAX);  // milos, with autocalibration
        clutch = map(clutch, clutchMin + dz, clutchMax - dz, 0, RX_AXIS_PHYS_MAX);
        hbrake = map(hbrake, hbrakeMin + dz, hbrakeMax - dz, 0, RY_AXIS_PHYS_MAX);
#else //if no autocalib
        accel = map(accel, 0 + dz, Z_AXIS_PHYS_MAX - dz, 0, Z_AXIS_PHYS_MAX); // milos, no autocalibration
        clutch = map(clutch, 0 + dz, RX_AXIS_PHYS_MAX - dz, 0, RX_AXIS_PHYS_MAX);
        hbrake = map(hbrake, 0 + dz, RY_AXIS_PHYS_MAX - dz, 0, RY_AXIS_PHYS_MAX);
#endif //end of autocalib
        accel = constrain(accel, 0, Z_AXIS_PHYS_MAX); // milos, limit axis ranges
        clutch = constrain(clutch, 0, RX_AXIS_PHYS_MAX);
        hbrake = constrain(hbrake, 0, RY_AXIS_PHYS_MAX);

#ifdef USE_LOAD_CELL // we use LC
        if (brake < bdz) { // milos, cut low values
          brake = 0;
        } else {
          brake = map(brake, bdz, Y_AXIS_PHYS_MAX + bdz, 0, Y_AXIS_PHYS_MAX); // milos, no autocalibration
          brake = constrain(brake, 0, Y_AXIS_PHYS_MAX); // milos
        }
#else // milos, when no LC
#ifdef  USE_AUTOCALIB
        brake = map(brake, brakeMin + dz, brakeMax - dz, 0, Y_AXIS_PHYS_MAX); // milos, with autocalibration
#else
        brake = map(brake, 0 + dz, 4095 - dz, 0, Y_AXIS_PHYS_MAX); // milos, no autocalibration
#endif
        brake = constrain(brake, 0, Y_AXIS_PHYS_MAX);

#endif //milos, end of USE_LOAD_CELL

        //milos, commented
        //shifterX = map(shifterX, 0, 1024, 0, 255);
        //shifterY = map(shifterY, 0, 1024, 0, 255);	// DEBUG H-SHIFTER

        button = readInputButtons();

        //SendInputReport((s16)turn, (u16)accel, (u16)brake, (u16)clutch, button);
        //SendInputReport((s16)turn, (u16)accel, (u16)brake, (u16)clutch, (u16)shifterX, (u16)shifterY, buttons); // original
        //SendInputReport((s32)turn, (u16)brake, (u16)accel, (u16)clutch, button); // milos, X, Y, Z, RX, button
        SendInputReport((s32)turn, (u16)brake, (u16)accel, (u16)clutch, (u16)hbrake, button); // milos, X, Y, Z, RX, RY, hat+button

#ifdef AVG_INPUTS //milos, added option see config.h
        ClearAnalogInputs();
#endif
        if (timeDiffConfigSerial >= CONFIG_SERIAL_PERIOD) {
          readSerial();
          last_ConfigSerial = now_micros;
        }
      }
      //SYNC_LED_LOW(); //milos
    }
#endif
  }
}
