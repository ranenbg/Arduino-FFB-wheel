#include "Config.h"
#include "debug.h"

//--------------------------------------------------------------------------------------------------------

u8 toUpper(u8 c) {
  if ((c >= 'a') && (c <= 'z'))
    return (c + 'A' - 'a');
  return (c);
}

void configCDC() { // milos, virtual serial port firmware configuration interface
  if (CONFIG_SERIAL.available() > 0) {
    u8 c = toUpper(CONFIG_SERIAL.read());
    //DEBUG_SERIAL.println(c);
    s32 temp, temp1;
    f32 wheelAngle;
    u8 ffb_temp;
    switch (c) {
      case 'U': // milos, send all firmware settings
        CONFIG_SERIAL.print(ROTATION_DEG);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configGeneralGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configDamperGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configFrictionGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configConstantGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configPeriodicGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configSpringGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configInertiaGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configCenterGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(configStopGain);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(MM_MIN_MOTOR_TORQUE);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(LC_scaling);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(effstate, DEC); //milos, desktop effects in decimal form
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(MM_MAX_MOTOR_TORQUE); //milos, send max torque as parameter
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.print(CPR); //milos, send CPR as parameter
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.println(pwmstate, DEC); //milos, send pwmstate byte in decimal form
        break;
      case 'V':
        CONFIG_SERIAL.print("fw-v");
        CONFIG_SERIAL.print(FIRMWARE_VERSION, DEC);
        // milos, firmware options
#ifdef USE_AUTOCALIB
        CONFIG_SERIAL.print("a");
#endif
#ifdef USE_TWOFFBAXIS
        CONFIG_SERIAL.print("b");
#endif
#ifdef USE_ZINDEX
        CONFIG_SERIAL.print("z");
#endif
#ifdef USE_CENTERBTN
        CONFIG_SERIAL.print("c");
#endif
#ifndef USE_QUADRATURE_ENCODER
        CONFIG_SERIAL.print("d"); // milos, if no optical encoder
#endif
#ifdef USE_AS5600
        CONFIG_SERIAL.print("w");
#endif
#ifdef USE_TCA9548
        CONFIG_SERIAL.print("u"); // milos, if tca9548
#endif
#ifdef USE_HATSWITCH
        CONFIG_SERIAL.print("h");
#endif
#ifdef USE_ADS1015
        CONFIG_SERIAL.print("s");
#endif
#ifdef AVG_INPUTS
        CONFIG_SERIAL.print("i");
#endif
#ifdef USE_BTNMATRIX
        CONFIG_SERIAL.print("t");
#endif
#ifdef USE_XY_SHIFTER
        CONFIG_SERIAL.print("f");
#endif
#ifdef USE_EXTRABTN
        CONFIG_SERIAL.print("e");
#endif
#ifdef USE_ANALOGFFBAXIS
        CONFIG_SERIAL.print("x");
#endif
#ifdef USE_SHIFT_REGISTER
        CONFIG_SERIAL.print("n");
#endif
#ifdef USE_SN74ALS166N
        CONFIG_SERIAL.print("r");
#endif
#ifdef USE_LOAD_CELL
        CONFIG_SERIAL.print("l");
#endif
#ifdef USE_MCP4725
        CONFIG_SERIAL.print("g");
#endif
#ifndef USE_EEPROM
        CONFIG_SERIAL.print("p");
#endif
#ifdef USE_PROMICRO
        CONFIG_SERIAL.print("m");
#endif
        CONFIG_SERIAL.print("\r\n");
        break;
      case 'S':
        CONFIG_SERIAL.println(brWheelFFB.state, DEC);
        break;
      case 'R':
        brWheelFFB.calibrate();
        break;
      case 'B': // milos, added to adjust brake load cell pressure
        ffb_temp = CONFIG_SERIAL.parseInt();
        LC_scaling = constrain(ffb_temp, 1, 255);
        CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_BRK_PRES, LC_scaling); // milos, update EEPROM
        break;
      case 'P': // milos, added to recalibrate pedals
#ifdef USE_AUTOCALIB
        accel.min = Z_AXIS_LOG_MAX, accel.max = 0;
        brake.min = Z_AXIS_LOG_MAX, brake.max = 0;
        clutch.min = RX_AXIS_LOG_MAX, clutch.max = 0;
        hbrake.min = RY_AXIS_LOG_MAX, hbrake.max = 0;
        CONFIG_SERIAL.println(1);
#else // if manual calib
        CONFIG_SERIAL.println(0);
#endif // end of autocalib
        break;
      case 'O': // milos, added to adjust optical encoder CPR
#ifdef USE_AS5600 // milos, with AS5600
#ifdef USE_TCA9548
        TcaChannelSel(baseTCA0, 0); // milos, select 1st i2C channel for AS5600(0x36) on x-axis
#endif // end of tca
        temp1 = as5600x.getCumulativePosition() - ROTATION_MID; // milos
#else // if no as5600
#ifdef USE_QUADRATURE_ENCODER
        temp1 = myEnc.Read() - ROTATION_MID + brWheelFFB.offset; // milos
#else // milos, if no digital encoders
        temp1 = 0;
#endif // end of quad enc
#endif // end of as5600
        temp = CONFIG_SERIAL.parseInt();
        temp = constrain(temp, 4, 600000); // milos, extended to 32bit (100000*6)
        wheelAngle = float(temp1) * float(ROTATION_DEG) / float(ROTATION_MAX); // milos, current wheel angle
        CPR = temp; // milos, update CPR
        ROTATION_MAX = int32_t(float(CPR) / 360.0 * float(ROTATION_DEG)); // milos, updated
        ROTATION_MID = ROTATION_MAX >> 1; // milos, updated, divide by 2
        temp1 = int32_t(wheelAngle * float(ROTATION_MAX) / float(ROTATION_DEG)); // milos, here we recover the old wheel angle
#ifdef USE_AS5600 // milos, with AS5600
#ifdef USE_TCA9548
        TcaChannelSel(baseTCA0, 0); // milos, select 1st i2C channel for AS5600(0x36) on x-axis
#endif // end of tca
        as5600x.resetCumulativePosition(temp1 + ROTATION_MID); // milos
#ifdef USE_TCA9548
        TcaChannelSel(baseTCA0, 1); // milos, select 2nd i2C channel for AS5600(0x36) on y-axis
        as5600y.resetCumulativePosition(temp1 + ROTATION_MID); // milos, 2nd as5600
#endif // end of tca
#else // if no as5600
#ifdef USE_QUADRATURE_ENCODER
        myEnc.Write(temp1 + ROTATION_MID - brWheelFFB.offset); // milos
#endif // end of quad enc
#endif // end of as5600
        CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_ENC_CPR, CPR); // milos, update EEPROM
        break;
      case 'C':
#ifdef USE_ZINDEX
        brWheelFFB.offset = ROTATION_MID - myEnc.Read();
        //CONFIG_SERIAL.println(brWheelFFB.offset); // milos, saving some bytes in flash memory
        CONFIG_SERIAL.println(1);
#else // if no zindex
#ifdef USE_AS5600 // milos, with AS5600
#ifdef USE_TCA9548
        TcaChannelSel(baseTCA0, 0); // milos, select 1st i2C channel for AS5600(0x36) on x-axis
#endif // end of tca
        as5600x.resetCumulativePosition(ROTATION_MID);
#ifdef USE_TCA9548
        TcaChannelSel(baseTCA0, 1); // milos, select 2nd i2C channel for AS5600(0x36) on y-axis
        as5600y.resetCumulativePosition(ROTATION_MID);
#endif // end of tca
#else // milos, if no as5600
#ifdef USE_QUADRATURE_ENCODER
        myEnc.Write(ROTATION_MID); // milos, just set to zero angle
#endif // end of quad enc
#endif // end of use as5600
        CONFIG_SERIAL.println(0);
#endif // end of use z index
        break;
      case 'Z': // milos, hard reset the z-index offset
#ifdef USE_ZINDEX
        brWheelFFB.offset = 0;
        SetParam(PARAM_ADDR_ENC_OFFSET, brWheelFFB.offset); // milos, update EEPROM right away
        CONFIG_SERIAL.println(1);
#else // if no zindex
        CONFIG_SERIAL.println(0);
#endif // end of zindex
        break;
      case 'G': // milos, set new rotation angle
#ifdef USE_AS5600 // milos, with AS5600
#ifdef USE_TCA9548
        TcaChannelSel(baseTCA0, 0); // milos, select 1st i2C channel for AS5600(0x36) on x-axis
#endif // end of tca
        temp1 = as5600x.getCumulativePosition() - ROTATION_MID; // milos
#else // if no as5600
#ifdef USE_QUADRATURE_ENCODER
        temp1 = myEnc.Read() - ROTATION_MID + brWheelFFB.offset; // milos
#else // milos, if no digital encoders
        temp1 = 0;
#endif // end of quad enc
#endif // end of as5600
        temp = CONFIG_SERIAL.parseInt();
        temp = constrain(temp, 30, 1800); // milos
        wheelAngle = float(temp1) * float(ROTATION_DEG) / float(ROTATION_MAX); // milos, current wheel angle
        ROTATION_DEG = temp; // milos, update degrees of rotation
        ROTATION_MAX = int32_t(float(CPR) / 360.0 * float(ROTATION_DEG)); // milos, updated
        ROTATION_MID = ROTATION_MAX >> 1; // milos, updated, divide by 2
        temp1 = int32_t(wheelAngle * float(ROTATION_MAX) / float(ROTATION_DEG)); // milos, here we recover the old wheel angle
#ifdef USE_AS5600 // milos, with AS5600
#ifdef USE_TCA9548
        TcaChannelSel(baseTCA0, 0); // milos, select 1st i2C channel for AS5600(0x36) on x-axis
#endif // end of tca
        as5600x.resetCumulativePosition(temp1 + ROTATION_MID); // milos, 1st as5600
#ifdef USE_TCA9548
        TcaChannelSel(baseTCA0, 1); // milos, select 2nd i2C channel for AS5600(0x36) on y-axis
        as5600y.resetCumulativePosition(temp1 + ROTATION_MID); // milos, 2nd as5600
#endif // end of tca
#else // if no as5600
#ifdef USE_QUADRATURE_ENCODER
        myEnc.Write(temp1 + ROTATION_MID - brWheelFFB.offset); // milos
#endif // end of quad enc
#endif // end of as5600
        CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_ROTATION_DEG, temp);// milos, update EEPROM
        break;
      case 'E': //milos, added - turn desktop effects and ffb monitor on/off
        ffb_temp = CONFIG_SERIAL.parseInt();
        ffb_temp = constrain(ffb_temp, 0, 255);
        for (uint8_t i = 0; i < 8; i++) { //milos, decode incomming number into individual bits
          bitWrite(effstate, i, bitRead(ffb_temp, i));
        }
        CONFIG_SERIAL.println(effstate, BIN);
        //CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_DSK_EFFC, effstate); // milos, update EEPROM
        break;
      case 'W': //milos, added - configure PWM settings and frequency
#ifdef USE_EEPROM
        ffb_temp = CONFIG_SERIAL.parseInt();
        ffb_temp = constrain(ffb_temp, 0, 255);
        for (uint8_t i = 0; i < 8; i++) { // milos, decode incomming number into individual bits
          bitWrite(pwmstate, i, bitRead(ffb_temp, i));
        }
        SetParam(PARAM_ADDR_PWM_SET, pwmstate); // milos, update EEPROM with new pwm settings
        temp = calcTOP(pwmstate) * minTorquePP; // milos, recalculate new min torque for curent min torque %
        SetParam(PARAM_ADDR_MIN_TORQ, temp); // milos, update min torque in EEPROM
        CONFIG_SERIAL.println(calcTOP(pwmstate));
        //CONFIG_SERIAL.println(1);
#else // milos, if no eeprom we can't configure pwm settings durring runtime, but we can set it manualy in setup loop
        CONFIG_SERIAL.println(0);
#endif // end of eeprom
        break;
      case 'H': // milos, added - configure the XY shifter calibration
#ifdef USE_XY_SHIFTER
        c = toUpper(CONFIG_SERIAL.read());
        switch (c) {
          case 'A':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 1023);
            shifter.cal[0] = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'B':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 1023);
            shifter.cal[1] = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'C':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 1023);
            shifter.cal[2] = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'D':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 1023);
            shifter.cal[3] = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'E':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 1023);
            shifter.cal[4] = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'F':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 255);
            shifter.cfg = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'G':
            CONFIG_SERIAL.print(shifter.cal[0]);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(shifter.cal[1]);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(shifter.cal[2]);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(shifter.cal[3]);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(shifter.cal[4]);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.println(shifter.cfg);
            break;
          case 'R':
            CONFIG_SERIAL.print(shifter.x);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.println(shifter.y);
            break;
        }
#else // if no xy shifter
        CONFIG_SERIAL.println(0);
#endif // end of xy shifter
        break;
      case 'Y': // milos, added - configure manual calibration for pedals
#ifndef USE_AUTOCALIB // milos, if manual pedal calibration
        c = toUpper(CONFIG_SERIAL.read());
        switch (c) {
          case 'A':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            brake.min = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'B':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            brake.max = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'C':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            accel.min = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'D':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            accel.max = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'E':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            clutch.min = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'F':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            clutch.max = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'G':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            hbrake.min = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'H':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            hbrake.max = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'R':
            CONFIG_SERIAL.print(brake.min);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(brake.max);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(accel.min);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(accel.max);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(clutch.min);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(clutch.max);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(hbrake.min);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.println(hbrake.max);
            break;
        }
#else // if autocalib
        CONFIG_SERIAL.println(0);
#endif // end of autocalib
        break;
      /*case 'Q': //milos, read and print out EEPROM contents
        uint8_t temp;
        for (uint8_t i = 0; i < 64; i++) {
          for (uint8_t j = 0; j < 16; j++) {
            GetParam(i * 16 + j, temp);
            if (j == 0) {
              Serial.print("0x");
              if (i < 16) Serial.print("0");
              Serial.print(i, HEX);
              Serial.print(": ");
            }
            if (temp < 16) Serial.print("0");
            if (j < 15) {
              Serial.print(temp, HEX);
              Serial.print(" ");
              if (j == 7) Serial.print(" ");
            } else {
              Serial.println(temp, HEX);
            }
          }
        }
        break;*/
      /*case 'W': //milos, clear EEPROM contents
        ClearEEPROMConfig();
        Serial.println("EEPROM cleared");
        break;*/
      case 'A': //milos, save all firmware settings in EEPROM
#ifdef USE_EEPROM
        SaveEEPROMConfig ();
        CONFIG_SERIAL.println(1);
#else // if no eeprom
        CONFIG_SERIAL.println(0);
#endif // end of eeprom
        break;
      case 'F':
        c = toUpper(CONFIG_SERIAL.read());
        switch (c) {
          case 'G':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configGeneralGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'C':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configConstantGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'D':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configDamperGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'F':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configFrictionGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'S':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configPeriodicGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'M':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configSpringGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'I':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configInertiaGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'A':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configCenterGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'B':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configStopGain = constrain(ffb_temp, 0, 255);
            CONFIG_SERIAL.println(1);
            break;
          case 'J':
            ffb_temp = CONFIG_SERIAL.parseInt();
            ffb_temp = constrain(ffb_temp, 0, 255); //milos
            minTorquePP = (f32)ffb_temp * 0.001; // milos, max is 25.5% or 0.255
            MM_MIN_MOTOR_TORQUE = (u16)(minTorquePP * (f32)MM_MAX_MOTOR_TORQUE); // milos, we can set it during run time
            CONFIG_SERIAL.println(1);
            break;
            /*case 'K': //milos, commented out, once set at compile time this must not be changed anymore
              temp = CONFIG_SERIAL.parseInt();
              if (temp > MM_MIN_MOTOR_TORQUE)
              {
                MM_MAX_MOTOR_TORQUE = constrain(temp, 1, TOP);
              }
              CONFIG_SERIAL.println(temp);  // milos
              break;*/
        }
        break;
    }
  }
}
