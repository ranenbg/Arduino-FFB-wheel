/* Arduino Leonardo Force Feedback Wheel firmware

  Copyright 2015  Etienne Saint-Paul  (esaintpaul [at] gameseed [dot] fr)
  Copyright 2017  Fernando Igor  (fernandoigor [at] msn [dot] com)
  Copyright 2018-2024  Milos Rankovic (ranenbg [at] gmail [dot] com)

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
//#include <digitalWriteFast.h>
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
#ifdef USE_ADS1015
#include <Adafruit_ADS1015.h> //milos, added
#endif
#include "USBDesc.h"
#include "ConfigHID.h"
#ifdef USE_MCP4725
#include <Adafruit_MCP4725.h> //milos, added
#endif
#ifdef USE_AS5600 //milos, added
#include "AS5600.h"
#endif

extern u8 valueglobal;

//--------------------------------------- Globals --------------------------------------------------------

/*u8 spi_buffer[4];
  u8 spi_bp = 0;
  u8 spi_data_ready = false;*/

b8 fault;
s16 accel, clutch, hbrake; // milos
#ifdef USE_XY_SHIFTER
s16 shifterX, shifterY; // milos
#endif
s32 brake; //milos, we need 32bit due to 24 bits on load cell ADC
s32 turn;
u16 accelMin, accelMax; // milos
s16 brakeMin, brakeMax; // milos, must be signed
u16 clutchMin, clutchMax; // milos
u16 hbrakeMin, hbrakeMax; // milos
//s16 accelMid, brakeMid, clutchMid, hbrakeMid; // milos
u32 button = 0; //milos, added

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

//u32 last_send = 0;
//u32 last_nextShift = 0;
u32 last_ConfigSerial = 0;
u32 last_refresh = 0;
s32 ffbX = 0;
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
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD i2C address
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

#ifdef USE_AS5600 // milos, added
AS5600L as5600(0x36); // uses default wire.h, milos added i2C address
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
  effstate = 0b00000001; //only desktop spring effect is enabled
#ifdef USE_LOAD_CELL
  LC_scaling = 45; // milos, brake pressure
#else
  LC_scaling = 128; // milos, FFB balance
#endif
#ifndef USE_MCP4725
  pwmstate = 0b00001001; // milos, PWM out enabled, phase correct, pwm+-, 16kHz, TOP 500
#else
  pwmstate = 0b10000000; // milos, DAC out enabled, DAC+- mode
#endif
  MM_MIN_MOTOR_TORQUE = 0;
  minTorquePP = 0;
#endif
  ROTATION_MAX = int32_t(float(CPR) / 360.0 * float(ROTATION_DEG)); // milos
  ROTATION_MID = ROTATION_MAX >> 1; // milos

#ifndef USE_AS5600
  //myEnc.Init(ROTATION_MID + brWheelFFB.offset, true); //ROTATION_MID + gCalibrator.mOffset); // milos, pullups enabled
  myEnc.Init(ROTATION_MID, true); // milos, pullups enabled, do not apply any encoder offset at this point
#endif

  InitInputs();
  FfbSetDriver(0);

#ifdef USE_VNH5019
  ms.init();
#endif

  InitPWM(); // milos, initialize PWM (or DAC) settings
  SetPWM(0); // milos, set PWM (or DAC) to 0 at startup

#ifdef USE_QUADRATURE_ENCODER
#ifndef USE_AS5600
  (CALIBRATE_AT_INIT ? brWheelFFB.calibrate() : myEnc.Write(ROTATION_MID)); // milos, allways set encoder at 0deg (ROTATION_MID) at startup
#endif
#endif

#ifdef USE_LCD //milos
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
  last_LC_scaling = LC_scaling; // milos

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
  Wire.begin(); // milos, D2-SDA and D3-SCL by default
#endif
  as5600.begin();
  as5600.setFastFilter(0); // milos, added to configure fast filter threshold (0-OFF is slow filter enable)
  as5600.setSlowFilter(0); // milos, added to configure slow filter or readout precision: 0-best(slowest), 3-worst(fastest)
  //as5600.setAddress(0x36); // milos, not needed, address already defined in function constructor
  //as5600.setDirection(AS5600_CLOCK_WISE); // milos, not needed, but DIR pin has to be on GND
  as5600.resetCumulativePosition(ROTATION_MID); // milos, initialize at 0deg at startup
#endif
}

//--------------------------------------------------------------------------------------------------------
void loop() {
#ifdef AVG_INPUTS //milos, added option see config.h
  if (asc < avgSamples) {
    ReadAnalogInputs(); // milos, get readings for averaging (only do it until we get all samples)
    asc++; // milos
  }
#endif

  now_micros = micros();
  {
    //time_diff = now_micros - last_send;
    //timeDiffNextShift = now_micros - last_nextShift;
    timeDiffConfigSerial = now_micros - last_ConfigSerial;

#ifdef USE_QUADRATURE_ENCODER
    if ((now_micros - last_refresh) >= CONTROL_PERIOD) {
#ifdef AVG_INPUTS //milos
      asc = 0; // milos, reset counter for averaging
#endif // end of avg_inputs
      last_refresh = now_micros;
      //SYNC_LED_HIGH(); // milos
#ifdef  USE_SHIFT_REGISTER
      for (uint8_t i = 0; i <= SHIFTS_NUM; i++) { // milos, added (read all states in one pass)
        nextInputState();  // milos, refresh state of shift-register and read the incoming bit
      }
#endif // end of shift register
#ifndef USE_AS5600 // milos, if AS5600 is not enabled quadrature encoder is used
      if (zIndexFound) {
        turn = myEnc.Read() - ROTATION_MID + brWheelFFB.offset; // milos, only apply z-index offset if z-index pulse is found
      } else {
        turn = myEnc.Read() - ROTATION_MID;
      }
#else
      // milos, to do - implement AS5600 angle readout here
      turn = as5600.getCumulativePosition() - ROTATION_MID;
#endif // end of as5600

#ifndef USE_ANALOGFFBAXIS
      ffbX = gFFB.CalcTorqueCommands(turn, 0); // milos, encoder raw units -inf,0,inf
#else // with analog ffb axis
      int16_t aRng = 511; // milos, scaled analog axis range to be used as FFB steering input
#ifdef AVG_INPUTS
      aRng = 2047; // if averaging of arduino 10bit analog inputs is enabled
#endif // end of avg inputs
#ifdef USE_ADS1015
      aRng = 1023; // if 12bit external ADC via ads1105 is enabled
#endif // end of ads1015
#ifndef USE_TWOFFBAXIS // milos, if 1 ffb axis
      if (indFFBAxis(effstate) == 1) {
        ffbX = gFFB.CalcTorqueCommands(map(brake, 0, Y_AXIS_PHYS_MAX, -aRng - 1, aRng), 0); // milos, FFB on Y-axis
      } else if (indFFBAxis(effstate) == 2) {
        ffbX = gFFB.CalcTorqueCommands(map(accel, 0, Z_AXIS_PHYS_MAX, -aRng - 1, aRng), 0); // milos, FFB on Z-axis
      } else if (indFFBAxis(effstate) == 3) {
        ffbX = gFFB.CalcTorqueCommands(map(clutch, 0, RX_AXIS_PHYS_MAX, -aRng - 1, aRng), 0); // milos, FFB on RX-axis
      } else if (indFFBAxis(effstate) == 4) {
        ffbX = gFFB.CalcTorqueCommands(map(hbrake, 0, RY_AXIS_PHYS_MAX, -aRng - 1, aRng), 0); // milos, FFB on RY-axis
      } else { // milos, fail safe
        ffbX = gFFB.CalcTorqueCommands(turn, 0); // milos, FFB on X-axis
      }
#else // milos, with 2 ffb axis
      ffbX = gFFB.CalcTorqueCommands(map(brake, 0, Y_AXIS_PHYS_MAX, -aRng - 1, aRng), map(accel, 0, Z_AXIS_PHYS_MAX, -aRng - 1, aRng)); // milos, X-FFB on Y-axis, Y-FFB on Z-axis
#endif // end of 2 ffb axis
#endif // end of analog ffb axis
      turn *= f32(X_AXIS_PHYS_MAX) / f32(ROTATION_MAX); // milos, conversion to physical units
      turn = constrain(turn, -MID_REPORT_X - 1, MID_REPORT_X); // milos, -32768,0,32767 scaled to signed 16bit range

      SetPWM(ffbX); // milos, FFB signal is generated as PWM (or analog DAC) output
      //SYNC_LED_LOW(); //milos
      // USB Report
      {
        //last_send = now_micros;
#ifdef AVG_INPUTS //milos, added option see config.h
        AverageAnalogInputs();				// Average readings
#endif
        /*accel = analog_inputs[ACCEL_INPUT]; // milos, commented
          brake = analog_inputs[BRAKE_INPUT];
          clutch = analog_inputs[CLUTCH_INPUT];*/

        /*ads0 = ads.readADC_Differential_0_1();  // milos, diff input between A0 and A1
          ads1 = ads.readADC_Differential_2_3();  // milos, diff input between A2 and A3
          ads2 = ads.readADC_SingleEnded(0);  // milos, single input A0*/

#ifdef USE_ADS1015 // milos, if you plan to use ADS1105 for all 3 pedals (no load cell), then change to readADC_SingleEnded
        accel = constrain(ads.readADC_SingleEnded(ACCEL_INPUT), 0 , 2047); //milos, Z axis, 11bit
        clutch = constrain(ads.readADC_SingleEnded(CLUTCH_INPUT), 0 , 2047); //milos, RX axis, 11bit
        hbrake = constrain(ads.readADC_SingleEnded(HBRAKE_INPUT), 0 , 2047); //milos, RY axis, 11bit
        //accel = constrain(ads.readADC_Differential_0_1()+2048, 0, 4095);  //milos, Z axis, 12bit
        //clutch = constrain(ads.readADC_Differential_2_3()+2048, 0, 4095); //milos, RX axis, 12bit

#else //if no ads
#ifdef AVG_INPUTS //milos, added option - we do not avg h-shifter axis
        accel = analog_inputs[ACCEL_INPUT];
#ifdef USE_PROMICRO
#ifndef USE_XY_SHIFTER
        clutch = analog_inputs[CLUTCH_INPUT];
        hbrake = analog_inputs[HBRAKE_INPUT];
#else // milos, if we use h-shifter on proMicro with avg inputs
        clutch = 0;
        hbrake = 0;
        shifterX = analogRead(CLUTCH_PIN); // milos
        shifterY = analogRead(HBRAKE_PIN); // milos
#endif // end of xy shifter
#else // for leonardo we can avg pedal inputs and also have h-shifter axis
        clutch = analog_inputs[CLUTCH_INPUT];
        hbrake = analog_inputs[HBRAKE_INPUT];
#ifdef USE_XY_SHIFTER
        shifterX = analogRead(SHIFTER_X_PIN); // milos
        shifterY = analogRead(SHIFTER_Y_PIN); // milos
#endif // end of h-shifter
#endif // end of proMicro
#else // if no avg
        accel = analogRead(ACCEL_PIN); // milos, Z axis
#ifndef USE_PROMICRO // milos, for Leonardo and Micro
#ifndef USE_EXTRABTN // milos, we can have clutch and hbrake only when not using extra buttons
        clutch = analogRead(CLUTCH_PIN); // milos, RX axis
        hbrake = analogRead(HBRAKE_PIN); // milos, RY axis
#else
        clutch = 0; // milos, RX axis
        hbrake = 0; // milos, RY axis
#endif // end of extra button
#ifdef USE_XY_SHIFTER // milos
        shifterX = analogRead(SHIFTER_X_PIN); // milos
        shifterY = analogRead(SHIFTER_Y_PIN); // milos
#endif // end of xy shifter
#else // if we use proMicro
#ifdef USE_XY_SHIFTER // milos, compromize - for proMicro with XY shifter, we can't have clutch and handbrake
        clutch = 0; // milos, RX axis
        hbrake = 0; // milos, RY axis
        shifterX = analogRead(CLUTCH_PIN); // milos, use clutch analog input instead
        shifterY = analogRead(HBRAKE_PIN); // milos, use handbrake analog input instead
#else // for proMicro, when no XY shifter
#ifndef USE_EXTRABTN // milos, only available if not using extra buttons
        clutch = analogRead(CLUTCH_PIN); // milos, RX axis
        hbrake = analogRead(HBRAKE_PIN); // milos, RY axis
#else // if using extra buttons
#ifndef USE_LOAD_CELL
        clutch = 0; // milos, RX axis unavailable when no lc
#else
        clutch = analogRead(CLUTCH_PIN); // milos, RX axis is available if we use lc
#endif // end of use lc
        hbrake = 0; // milos, RY axis is allways unavailable
#endif // end of extra button
#endif // end of xy shifter
#endif // end proMicro
#endif // end of avg
#endif // end of ads

#ifdef USE_LOAD_CELL // milos, when use LC
        if (LC_scaling != last_LC_scaling) {
          LoadCell.setCalFactor(0.25 * float(LC_scaling)); // user set calibration factor (float) // milos, apply only if changed (through serial interface)
          last_LC_scaling = LC_scaling;
          LoadCell.tare(); //milos, zero out the measurement
        }
        //update() should be called at least as often as HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS
        //longer delay in sketch will reduce effective sample rate (be carefull with delay() in loop)
        LoadCell.update(); // milos, I have configured mine for 80Hz reading by applying 5V at pin15 of HX711 chip (by default it's 10Hz, pin15 grounded)
        brake = LoadCell.getData(); // milos, read smoothed data from LC (running average of 1 samples, see HX711_ADC.h I modded it)
#else // milos, when no LC
#ifdef USE_ADS1015
        brake = constrain(ads.readADC_SingleEnded(BRAKE_INPUT), 0 , 2047); // milos, Y axis, 11bit
#else // if no ads
#ifdef AVG_INPUTS // milos, added option
        brake = analog_inputs[BRAKE_INPUT];
#else // if no avg
        brake = analogRead(BRAKE_PIN); // milos, Y axis
#endif // end of avg
#endif // end of ads
#endif // end of lc


#ifdef  USE_AUTOCALIB // milos, update limits for autocalibration
        if (accel < accelMin) accelMin = accel;
        if (accel > accelMax) accelMax = accel;
        if (brake < brakeMin) brakeMin = brake;
        if (brake > brakeMax) brakeMax = brake;
        if (clutch < clutchMin) clutchMin = clutch;
        if (clutch > clutchMax) clutchMax = clutch;
        if (hbrake < hbrakeMin) hbrakeMin = hbrake;
        if (hbrake > hbrakeMax) hbrakeMax = hbrake;
#endif // end of autocalib
#ifdef USE_AVGINPUTS
        // milos, update calibration limits for increased axis resolution due to averaging (depends on num of samples)
        accelMin *= avgSamples;
        accelMax *= avgSamples;
#ifndef USE_LOADCELL
        brakeMin *= avgSamples;
        brakeMax *= avgSamples;
#endif
        clutchMin *= avgSamples;
        clutchMax *= avgSamples;
        hbrakeMin *= avgSamples;
        hbrakeMax *= avgSamples;
#endif // end of avg inputs
        // milos, rescale all axis according to a new manual calibration
        accel = map(accel, accelMin + dz, accelMax - dz, 0, Z_AXIS_PHYS_MAX);  // milos, with manual calibration
        clutch = map(clutch, clutchMin + dz, clutchMax - dz, 0, RX_AXIS_PHYS_MAX);
        hbrake = map(hbrake, hbrakeMin + dz, hbrakeMax - dz, 0, RY_AXIS_PHYS_MAX);
        accel = constrain(accel, 0, Z_AXIS_PHYS_MAX); // milos, constrain axis ranges
        clutch = constrain(clutch, 0, RX_AXIS_PHYS_MAX);
        hbrake = constrain(hbrake, 0, RY_AXIS_PHYS_MAX);

#ifdef USE_LOAD_CELL // we use LC
        if (brake < bdz) { // milos, cut low values
          brake = 0;
        } else {
          brake = map(brake, bdz, Y_AXIS_PHYS_MAX + bdz, 0, Y_AXIS_PHYS_MAX); // milos, no autocalibration
        }
#else // milos, when no LC
        brake = map(brake, brakeMin + dz, brakeMax - dz, 0, Y_AXIS_PHYS_MAX); // milos, for both manual and auto cal
#endif //milos, end of USE_LOAD_CELL
        brake = constrain(brake, 0, Y_AXIS_PHYS_MAX); // milos

        button = readInputButtons(); // milos, reads all buttons including matrix and hat switch

#ifdef USE_XY_SHIFTER // milos, added
        if ((bitRead(sConfig, 2))) shifterX = 1023 - shifterX; // milos, invert shifter X-axis
        if ((bitRead(sConfig, 3))) shifterY = 1023 - shifterY; // milos, invert shifter Y-axis
        button = decodeXYshifter(button, shifterX, shifterY); // milos, added - convert analog XY shifter values into last 8 buttons
#endif
        //SendInputReport((s16)turn, (u16)accel, (u16)brake, (u16)clutch, button);
        //SendInputReport((s16)turn, (u16)accel, (u16)brake, (u16)clutch, (u16)shifterX, (u16)shifterY, buttons); // original
        //SendInputReport((s32)turn, (u16)brake, (u16)accel, (u16)clutch, button); // milos, X, Y, Z, RX, button
        SendInputReport(turn + MID_REPORT_X + 1, brake, accel, clutch, hbrake, button); // milos, X, Y, Z, RX, RY, hat+button; (0-65535) X-axis range, center at 32768

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
#endif // end of use quad encoder
  }
}
