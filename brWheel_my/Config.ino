#include "Config.h"
#ifdef USE_EEPROM
#include <EEPROM.h> //milos, uncommented
#endif

//-------------------------------------------------------------------------------------------------------

//milos, uncommented
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
    EEPROM.update(offset + i, addr_to[i]); //milos, re-writes only when neccessary
  }
#endif
}

void SetDefaultConfig () { //milos - sets default values
  u16 val;
  s32 val32;
  u8 ffb; //milos, added
  val = VERSION;
  SetParam(PARAM_ADDR_VERSION, val);
  val32 = 0;
  SetParam(PARAM_ADDR_OFFSET, val32);
  val = 1080; //milos, default degrees of rotation
  SetParam(PARAM_ADDR_ROTATION_DEG, val); //milos, added
  ffb = 100; //milos, added
  SetParam(PARAM_ADDR_GEN_GAIN, ffb); //milos, added
  SetParam(PARAM_ADDR_CNT_GAIN, ffb); //milos, added
  SetParam(PARAM_ADDR_PER_GAIN, ffb); //milos, added
  SetParam(PARAM_ADDR_STP_GAIN, ffb); //milos, added
  SetParam(PARAM_ADDR_SPR_GAIN, ffb); //milos, added
  ffb = 50; //milos, added
  SetParam(PARAM_ADDR_DMP_GAIN, ffb); //milos, added
  SetParam(PARAM_ADDR_FRC_GAIN, ffb); //milos, added
  SetParam(PARAM_ADDR_INR_GAIN, ffb); //milos, added
  ffb = 70; //milos, added
  SetParam(PARAM_ADDR_CTS_GAIN, ffb); //milos, added
  val = 0; //milos, added
  SetParam(PARAM_ADDR_MIN_TORQ, val); //milos, added
  val = 500; //milos, for PWM signals
  SetParam(PARAM_ADDR_MAX_TORQ, val); //milos, added
  val = 4095; //milos, for 12bit DAC
  SetParam(PARAM_ADDR_MAX_DAC, val); //milos, added
#ifdef USE_LOAD_CELL
  val = 45; //milos, default max brake pressure
#else
  val = 128; //milos, FFB balance center value
#endif
  SetParam(PARAM_ADDR_BRK_PRES, val); //milos, added
  ffb = 0b00000001; //milos, added, autocenter spring on
  SetParam(PARAM_ADDR_DSK_EFFC, ffb); //milos, added
  val32 = 2400; //milos, default CPR value (here you can set a new one)
  SetParam(PARAM_ADDR_ENC_CPR, val32); //milos, added
#ifndef USE_MCP4725
  ffb = 0b00001001; // milos, PWM out enabled, phase correct, pwm+-, 16kHz, TOP 500
#else
  ffb = 0b10000000; //milos, DAC out enabled, DAC+- mode
#endif
  SetParam(PARAM_ADDR_PWM_SET, ffb); //milos, added
#ifdef USE_XY_SHIFTER
  val = 255;
  SetParam(PARAM_ADDR_SHFT_X0, val); //milos, added
  val = 511;
  SetParam(PARAM_ADDR_SHFT_X1, val); //milos, added
  val = 767;
  SetParam(PARAM_ADDR_SHFT_X2, val); //milos, added
  val = 255;
  SetParam(PARAM_ADDR_SHFT_Y0, val); //milos, added
  val = 511;
  SetParam(PARAM_ADDR_SHFT_Y1, val); //milos, added
  val = 0; // milos, 0b00000000 - default is 6 gear H shifter 
  SetParam(PARAM_ADDR_SHFT_CFG, val); //milos, added
#endif
}

void SetEEPROMConfig () { //milos, changed VERSION to 16bit from 32bit
  u16 value;
  GetParam(PARAM_ADDR_VERSION, value);
  if (value != VERSION) {							// First time run, or version changed, so put default values for safety
    //ClearEEPROMConfig(); //milos, clear EEPROM before loading defaults
    SetDefaultConfig();
  }
}

void LoadEEPROMConfig () { //milos, added - updates all ffb parameters from EEPROM
  GetParam(PARAM_ADDR_ROTATION_DEG, ROTATION_DEG);
#ifdef USE_ZINDEX
  GetParam(PARAM_ADDR_OFFSET, brWheelFFB.offset);
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
  minTorquePP = (f32)MM_MIN_MOTOR_TORQUE / (f32)MM_MAX_MOTOR_TORQUE;
  GetParam(PARAM_ADDR_PWM_SET, pwmstate);
#ifdef USE_XY_SHIFTER
  GetParam(PARAM_ADDR_SHFT_X0, sCal[0]); //milos, added
  GetParam(PARAM_ADDR_SHFT_X1, sCal[1]); //milos, added
  GetParam(PARAM_ADDR_SHFT_X2, sCal[2]); //milos, added
  GetParam(PARAM_ADDR_SHFT_Y0, sCal[3]); //milos, added
  GetParam(PARAM_ADDR_SHFT_Y1, sCal[4]); //milos, added
  GetParam(PARAM_ADDR_SHFT_CFG, sConfig); //milos, added
#endif
}

void SaveEEPROMConfig () { //milos, added - saves all ffb parameters in EEPROM
  SetParam(PARAM_ADDR_ROTATION_DEG, ROTATION_DEG);
#ifdef USE_ZINDEX
  SetParam(PARAM_ADDR_OFFSET, brWheelFFB.offset);
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
#ifdef USE_XY_SHIFTER
  SetParam(PARAM_ADDR_SHFT_X0, sCal[0]); //milos, added
  SetParam(PARAM_ADDR_SHFT_X1, sCal[1]); //milos, added
  SetParam(PARAM_ADDR_SHFT_X2, sCal[2]); //milos, added
  SetParam(PARAM_ADDR_SHFT_Y0, sCal[3]); //milos, added
  SetParam(PARAM_ADDR_SHFT_Y1, sCal[4]); //milos, added
  SetParam(PARAM_ADDR_SHFT_CFG, sConfig); //milos, added
#endif
}

void ClearEEPROMConfig() { //milos, added - clears EEPROM (1KB on ATmega32U4)
  uint8_t zero;
  zero = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    SetParam(i, zero);
  }
}
