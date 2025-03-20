#include "Config.h"
#ifdef USE_EEPROM
#include <EEPROM.h> // milos, re-implemented
#endif

//-------------------------------------------------------------------------------------------------------

void getParam (u16 offset, u8 *addr_to, u8 size) {
#ifdef USE_EEPROM
  for (u8 i = 0; i < size; i++) {
    addr_to[i] = EEPROM.read(offset + i);
  }
#endif
}

void setParam (u16 offset, u8 *addr_to, u8 size) {
#ifdef USE_EEPROM
  for (u8 i = 0; i < size; i++) {
    //EEPROM.write(offset + i, addr_to[i]);
    EEPROM.update(offset + i, addr_to[i]); //milos, re-write only when neccessary
  }
#endif
}

void SetDefaultEEPROMConfig() { // milos - store default firmware settings in EEPROM
  u16 v16;
  s32 v32;
  u8 v8;
  v16 = FIRMWARE_VERSION;
  SetParam(PARAM_ADDR_FW_VERSION, v16);
  v32 = 0;
  SetParam(PARAM_ADDR_ENC_OFFSET, v32);
  v16 = 1080; //milos, default degrees of rotation
  SetParam(PARAM_ADDR_ROTATION_DEG, v16); //milos, added
  v8 = 100; //milos, added
  SetParam(PARAM_ADDR_GEN_GAIN, v8); //milos, added
  SetParam(PARAM_ADDR_CNT_GAIN, v8); //milos, added
  SetParam(PARAM_ADDR_PER_GAIN, v8); //milos, added
  SetParam(PARAM_ADDR_STP_GAIN, v8); //milos, added
  SetParam(PARAM_ADDR_SPR_GAIN, v8); //milos, added
  v8 = 50; //milos, added
  SetParam(PARAM_ADDR_DMP_GAIN, v8); //milos, added
  SetParam(PARAM_ADDR_FRC_GAIN, v8); //milos, added
  SetParam(PARAM_ADDR_INR_GAIN, v8); //milos, added
  v8 = 70; //milos, added
  SetParam(PARAM_ADDR_CTS_GAIN, v8); // milos, added
  v16 = 0; // milos, added
  SetParam(PARAM_ADDR_MIN_TORQ, v16); //milos, added
  v16 = 2047; // milos, for PWM signals
  SetParam(PARAM_ADDR_MAX_TORQ, v16); //milos, added
  v16 = 4095; // milos, for 12bit DAC
  SetParam(PARAM_ADDR_MAX_DAC, v16); // milos, added
#ifdef USE_LOAD_CELL
  v8 = 45; // milos, default max brake pressure
#else
  v8 = 128; // milos, ffb balance center value
#endif
  SetParam(PARAM_ADDR_BRK_PRES, v8); // milos, added
  v8 = 0b00000001; // milos, added, autocenter spring on
  SetParam(PARAM_ADDR_DSK_EFFC, v8); // milos, added
#ifndef USE_AS5600
  v32 = 2400; // milos, default CPR value for optical encoder (this is for 600PPR)
#else
  v32 = 4096; // milos, default CPR value for 12bit magnetic encoder AS5600
#endif
  SetParam(PARAM_ADDR_ENC_CPR, v32); // milos, added
#ifndef USE_MCP4725
  v8 = 0b00001100; // milos, PWM out enabled, fast pwm, pwm+-, 7.8kHz, TOP 11bit (2047)
#else
  v8 = 0b10000000; // milos, DAC out enabled, DAC+- mode
#endif
  SetParam(PARAM_ADDR_PWM_SET, v8); // milos, added
#ifdef USE_XY_SHIFTER
  v16 = 255;
  SetParam(PARAM_ADDR_SHFT_X0, v16); // milos, added
  v16 = 511;
  SetParam(PARAM_ADDR_SHFT_X1, v16); // milos, added
  v16 = 767;
  SetParam(PARAM_ADDR_SHFT_X2, v16); // milos, added
  v16 = 255;
  SetParam(PARAM_ADDR_SHFT_Y0, v16); // milos, added
  v16 = 511;
  SetParam(PARAM_ADDR_SHFT_Y1, v16); // milos, added
  v16 = 0; // milos, 0b00000000 - default is rev gear in 6th, no inv x-axis, no inv y-axis, no inv rev gear button
  SetParam(PARAM_ADDR_SHFT_CFG, v16); // milos, added
#endif // end of xy shifter
#ifndef USE_AUTOCALIB //milos, added - load default min/max manual cal values for all pedal axis
  v16 = 0;
  SetParam(PARAM_ADDR_ACEL_LO, v16);
  SetParam(PARAM_ADDR_BRAK_LO, v16);
  SetParam(PARAM_ADDR_CLUT_LO, v16);
  SetParam(PARAM_ADDR_HBRK_LO, v16);
  v16 = maxCal;
  SetParam(PARAM_ADDR_ACEL_HI, v16);
  SetParam(PARAM_ADDR_BRAK_HI, v16);
  SetParam(PARAM_ADDR_CLUT_HI, v16);
  SetParam(PARAM_ADDR_HBRK_HI, v16);
#endif // end of autocalib
}

void SetEEPROMConfig() { // milos, changed FIRMWARE_VERSION to 16bit from 32bit
  u16 v16;
  GetParam(PARAM_ADDR_FW_VERSION, v16);
  if (v16 != FIRMWARE_VERSION) { // milos, first time run, or version change - set default values for safety
    //ClearEEPROMConfig(); // milos, clear EEPROM before loading defaults
    SetDefaultEEPROMConfig(); // milos, set default firmware settings
  }
}

void LoadEEPROMConfig () { //milos, added - updates all v8 parameters from EEPROM
  GetParam(PARAM_ADDR_ROTATION_DEG, ROTATION_DEG);
#ifdef USE_ZINDEX
  GetParam(PARAM_ADDR_ENC_OFFSET, brWheelFFB.offset);
#endif
  GetParam(PARAM_ADDR_ENC_CPR, CPR);
  GetParam(PARAM_ADDR_GEN_GAIN, configGeneralGain);
  GetParam(PARAM_ADDR_DMP_GAIN, configDamperGain);
  GetParam(PARAM_ADDR_FRC_GAIN, configFrictionGain);
  GetParam(PARAM_ADDR_CNT_GAIN, configConstantGain);
  GetParam(PARAM_ADDR_PER_GAIN, configPeriodicGain);
  GetParam(PARAM_ADDR_SPR_GAIN, configSpringGain);
  GetParam(PARAM_ADDR_INR_GAIN, configInertiaGain);
  GetParam(PARAM_ADDR_CTS_GAIN, configCenterGain);
  GetParam(PARAM_ADDR_STP_GAIN, configStopGain);
  GetParam(PARAM_ADDR_BRK_PRES, LC_scaling);
  GetParam(PARAM_ADDR_DSK_EFFC, effstate);
  GetParam(PARAM_ADDR_MIN_TORQ, MM_MIN_MOTOR_TORQUE);
  GetParam(PARAM_ADDR_MAX_TORQ, MM_MAX_MOTOR_TORQUE);
  GetParam(PARAM_ADDR_MAX_DAC, MAX_DAC);
#ifdef USE_MCP4725
  MM_MAX_MOTOR_TORQUE = MAX_DAC;
#endif
  GetParam(PARAM_ADDR_PWM_SET, pwmstate);
#ifdef USE_XY_SHIFTER
  GetParam(PARAM_ADDR_SHFT_X0, shifter.cal[0]); //milos, added
  GetParam(PARAM_ADDR_SHFT_X1, shifter.cal[1]); //milos, added
  GetParam(PARAM_ADDR_SHFT_X2, shifter.cal[2]); //milos, added
  GetParam(PARAM_ADDR_SHFT_Y0, shifter.cal[3]); //milos, added
  GetParam(PARAM_ADDR_SHFT_Y1, shifter.cal[4]); //milos, added
  GetParam(PARAM_ADDR_SHFT_CFG, shifter.cfg); //milos, added
#endif
#ifdef USE_AUTOCALIB // milos, added - reset autocalibration
  accel.min = Z_AXIS_LOG_MAX;
  accel.max = 0;
  brake.min = s16(Z_AXIS_LOG_MAX);
  brake.max = 0;
  clutch.min = RX_AXIS_LOG_MAX;
  clutch.max = 0;
  hbrake.min = RY_AXIS_LOG_MAX;
  hbrake.max = 0;
#else //milos, load min/max manual cal values from EEPROM
  GetParam(PARAM_ADDR_ACEL_LO, accel.min);
  GetParam(PARAM_ADDR_ACEL_HI, accel.max);
  GetParam(PARAM_ADDR_BRAK_LO, brake.min);
  GetParam(PARAM_ADDR_BRAK_HI, brake.max);
  GetParam(PARAM_ADDR_CLUT_LO, clutch.min);
  GetParam(PARAM_ADDR_CLUT_HI, clutch.max);
  GetParam(PARAM_ADDR_HBRK_LO, hbrake.min);
  GetParam(PARAM_ADDR_HBRK_HI, hbrake.max);
#endif
}

void SaveEEPROMConfig () { //milos, added - saves all v8 parameters in EEPROM
  SetParam(PARAM_ADDR_ROTATION_DEG, ROTATION_DEG);
#ifdef USE_ZINDEX
  SetParam(PARAM_ADDR_ENC_OFFSET, brWheelFFB.offset);
#endif
  SetParam(PARAM_ADDR_ENC_CPR, CPR);
  SetParam(PARAM_ADDR_GEN_GAIN, configGeneralGain);
  SetParam(PARAM_ADDR_DMP_GAIN, configDamperGain);
  SetParam(PARAM_ADDR_FRC_GAIN, configFrictionGain);
  SetParam(PARAM_ADDR_CNT_GAIN, configConstantGain);
  SetParam(PARAM_ADDR_PER_GAIN, configPeriodicGain);
  SetParam(PARAM_ADDR_SPR_GAIN, configSpringGain);
  SetParam(PARAM_ADDR_INR_GAIN, configInertiaGain);
  SetParam(PARAM_ADDR_CTS_GAIN, configCenterGain);
  SetParam(PARAM_ADDR_STP_GAIN, configStopGain);
  SetParam(PARAM_ADDR_BRK_PRES, LC_scaling);
  SetParam(PARAM_ADDR_DSK_EFFC, effstate);
  SetParam(PARAM_ADDR_MIN_TORQ, MM_MIN_MOTOR_TORQUE);
  SetParam(PARAM_ADDR_MAX_TORQ, MM_MAX_MOTOR_TORQUE);
  SetParam(PARAM_ADDR_MAX_DAC, MAX_DAC);
  //SetParam(PARAM_ADDR_PWM_SET, pwmstate); // milos, do not save it with command A (we do it with W instead)
#ifdef USE_XY_SHIFTER //milos, added - save curent limits for xy shifter calibration and config
  SetParam(PARAM_ADDR_SHFT_X0, shifter.cal[0]);
  SetParam(PARAM_ADDR_SHFT_X1, shifter.cal[1]);
  SetParam(PARAM_ADDR_SHFT_X2, shifter.cal[2]);
  SetParam(PARAM_ADDR_SHFT_Y0, shifter.cal[3]);
  SetParam(PARAM_ADDR_SHFT_Y1, shifter.cal[4]);
  SetParam(PARAM_ADDR_SHFT_CFG, shifter.cfg);
#endif // end of xy shifter
#ifndef USE_AUTOCALIB //milos, added - save current manual pedal calibration config
  SetParam(PARAM_ADDR_ACEL_LO, accel.min);
  SetParam(PARAM_ADDR_ACEL_HI, accel.max);
  SetParam(PARAM_ADDR_BRAK_LO, brake.min);
  SetParam(PARAM_ADDR_BRAK_HI, brake.max);
  SetParam(PARAM_ADDR_CLUT_LO, clutch.min);
  SetParam(PARAM_ADDR_CLUT_HI, clutch.max);
  SetParam(PARAM_ADDR_HBRK_LO, hbrake.min);
  SetParam(PARAM_ADDR_HBRK_HI, hbrake.max);
#endif
}

void ClearEEPROMConfig() { //milos, added - clears EEPROM (1KB on ATmega32U4)
#ifdef USE_EEPROM
  uint8_t zero;
  zero = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    SetParam(i, zero);
  }
#endif
}

int32_t SetCPR(uint32_t newCpr, int32_t encRead) { // milos, added - update encoder CPR (for both optical and magnetic), returns new encoder possition
  int32_t temp1;
  newCpr = constrain(newCpr, 4, 600000); // milos, extended to 32bit (100000*6)
#ifdef USE_AS5600 // milos, with AS5600
  temp1 = encRead - ROTATION_MID; // milos
#else // if no as5600
#ifdef USE_QUADRATURE_ENCODER
  temp1 = encRead - ROTATION_MID + brWheelFFB.offset; // milos
#endif // end of quad enc
#endif // end of as5600
  float wheelAngle = float(newCpr) * float(ROTATION_DEG) / float(ROTATION_MAX); // milos, current wheel angle
  CPR = newCpr; // milos, update CPR
  ROTATION_MAX = int32_t(float(CPR) / 360.0 * float(ROTATION_DEG)); // milos, updated
  ROTATION_MID = ROTATION_MAX >> 1; // milos, updated, divide by 2
  temp1 = int32_t(wheelAngle * float(ROTATION_MAX) / float(ROTATION_DEG)); // milos, here we recover the old wheel angle
#ifdef USE_AS5600 // milos, with AS5600
  temp1 += ROTATION_MID; // milos
#else // if no as5600
#ifdef USE_QUADRATURE_ENCODER
  temp1 += ROTATION_MID - brWheelFFB.offset; // milos
#endif // end of quad enc
#endif // end of as5600
  return temp1;
}

// arduino's map function works only up to range of int16_t variable (-32k,32k)
// I wrote mine that can handle 32bit variables or int32_t
int32_t myMap (int32_t value, int32_t x0, int32_t x1, int32_t y0, int32_t y1) {
  return (y0 + value * (y1 - y0) / (x1 - x0));
}
