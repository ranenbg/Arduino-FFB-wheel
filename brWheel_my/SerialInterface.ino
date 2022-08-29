#include "Config.h"
#include "debug.h"

//--------------------------------------------------------------------------------------------------------

u8 toUpper(u8 c)
{
  if ((c >= 'a') && (c <= 'z'))
    return (c + 'A' - 'a');
  return (c);
}

void readSerial() {
  if (CONFIG_SERIAL.available() > 0)
  {
    u8 c = toUpper(CONFIG_SERIAL.read());
    //DEBUG_SERIAL.println(c);
    s32 temp, temp1;
    f32 wheelAngle;
    u8 ffb_temp;
    switch (c) {
      case 'U': // milos, show all FFB parameters
        //CONFIG_SERIAL.println("Command      FFB parameters"); // milos
        //CONFIG_SERIAL.print(" [G]       degrees of rotation:   ");
        CONFIG_SERIAL.print(ROTATION_DEG);
        CONFIG_SERIAL.print(' ');
        //CONFIG_SERIAL.print(" [FK]    min PWM:               ");
        //CONFIG_SERIAL.println(MM_MIN_MOTOR_TORQUE);
        //CONFIG_SERIAL.print(" [FJ]    max PWM:               ");
        //CONFIG_SERIAL.println(MM_MAX_MOTOR_TORQUE);
        //CONFIG_SERIAL.print(" [FG]    overal gain:           ");
        CONFIG_SERIAL.print(configGeneralGain);
        CONFIG_SERIAL.print(' ');
        //CONFIG_SERIAL.print(" [FD]    damper gain:           ");
        CONFIG_SERIAL.print(configDamperGain);
        CONFIG_SERIAL.print(' ');
        //CONFIG_SERIAL.print(" [FF]    friction gain:         ");
        CONFIG_SERIAL.print(configFrictionGain);
        CONFIG_SERIAL.print(' ');
        //CONFIG_SERIAL.print(" [FC]    const force gain:      ");
        CONFIG_SERIAL.print(configConstantGain);
        CONFIG_SERIAL.print(' ');
        //CONFIG_SERIAL.print(" [FS]    sine gain:             ");
        CONFIG_SERIAL.print(configPeriodicGain);
        CONFIG_SERIAL.print(' ');
        //CONFIG_SERIAL.print(" [FM]    spring gain:           ");
        CONFIG_SERIAL.print(configSpringGain);
        CONFIG_SERIAL.print(' ');
        //CONFIG_SERIAL.print(" [FI]    inertia gain:          ");
        CONFIG_SERIAL.print(configInertiaGain);
        CONFIG_SERIAL.print(' ');
        //CONFIG_SERIAL.print(" [FA]    centering spring gain: ");
        CONFIG_SERIAL.print(configCenterGain);
        CONFIG_SERIAL.print(' ');
        //CONFIG_SERIAL.print(" [FB]    edge stop gain:        ");
        CONFIG_SERIAL.print(configStopGain);
        CONFIG_SERIAL.print(' ');
        //CONFIG_SERIAL.print(" [FJ]    min torque %PWM:        ");
        CONFIG_SERIAL.print(MM_MIN_MOTOR_TORQUE);
        CONFIG_SERIAL.print(' ');
        //CONFIG_SERIAL.print("Brake pressure [1-255]: ");
        CONFIG_SERIAL.print(LC_scaling);
        CONFIG_SERIAL.print(' ');
        //CONFIG_SERIAL.print("Desktop effects code: ");
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
        CONFIG_SERIAL.print(VERSION, DEC);
        // milos, firmware options
#ifdef USE_AUTOCALIB
        CONFIG_SERIAL.print("a");
#endif
#ifdef USE_ZINDEX
        CONFIG_SERIAL.print("z");
#else
#ifndef USE_ADS1015
#ifndef USE_MCP4725
#ifdef USE_CENTERBTN
        CONFIG_SERIAL.print("c");
#endif // end of centerbtn
#endif // end of mcp
#endif // end of ads
#endif // end of zindex
#ifdef USE_HATSWITCH
        CONFIG_SERIAL.print("h");
#endif
#ifdef USE_ADS1105
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
        //CONFIG_SERIAL.print("Brake pressure [1-255]: ");
        //CONFIG_SERIAL.println(LC_scaling);
        CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_BRK_PRES, LC_scaling);//milos, update EEPROM
        break;
      case 'P': // milos, added to recalibrate pedals
#ifdef USE_AUTOCALIB
        accelMin = Z_AXIS_LOG_MAX, accelMax = 0;
        brakeMin = Z_AXIS_LOG_MAX, brakeMax = 0;
        clutchMin = RX_AXIS_LOG_MAX, clutchMax = 0;
        hbrakeMin = RY_AXIS_LOG_MAX, hbrakeMax = 0;
        //CONFIG_SERIAL.println("ok");
        CONFIG_SERIAL.println(1);
#else
        //CONFIG_SERIAL.println("na");
        CONFIG_SERIAL.println(0);
#endif
        break;
      case 'O': // milos, added to adjust optical encoder CPR
        temp = CONFIG_SERIAL.parseInt();
        temp = constrain(temp, 4, 600000); //milos, extended to 32bit (100000*6)
        temp1 = myEnc.Read() - ROTATION_MID + brWheelFFB.offset; // milos
        wheelAngle = float(temp1) * float(ROTATION_DEG) / float(ROTATION_MAX); // milos, current wheel angle
        CPR = temp; // milos, update CPR
        ROTATION_MAX = int32_t(float(temp) / 360.0 * float(ROTATION_DEG)); // milos, updated
        ROTATION_MID = ROTATION_MAX / 2; // milos, updated
        //brWheelFFB.offset = 0; //milos
        temp1 = int32_t(wheelAngle * float(ROTATION_MAX) / float(ROTATION_DEG)); // milos, here we recover the old wheel angle
        myEnc.Write(ROTATION_MID + temp1 - brWheelFFB.offset); // milos
        //CONFIG_SERIAL.print("encoder CPR[4-600000]: ");
        //CONFIG_SERIAL.println(temp);
        CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_ENC_CPR, CPR);//milos, update EEPROM
        break;
      case 'C':
        //temp1 = myEnc.Read() - ROTATION_MID - brWheelFFB.offset;  // milos
        //temp1 = (temp1 *  X_AXIS_PHYS_MAX) / ROTATION_MAX; // milos
        //myEnc.Write(ROTATION_MID + brWheelFFB.offset);
        //CONFIG_SERIAL.print("Wheel centered from position: "); // milos
        //CONFIG_SERIAL.println(temp1);  // milos
#ifdef USE_ZINDEX
        brWheelFFB.offset = -myEnc.Read() + ROTATION_MID;
        CONFIG_SERIAL.println(brWheelFFB.offset);
#else
        //brWheelFFB.offset = 0;
        myEnc.Write(ROTATION_MID); // milos
        CONFIG_SERIAL.println(1);
#endif
        break;
      case 'G': // milos, this was not working, fixed now
        temp = CONFIG_SERIAL.parseInt();
        temp = constrain(temp, 30, 1800); // milos
        temp1 = myEnc.Read() - ROTATION_MID + brWheelFFB.offset; // milos
        wheelAngle = float(temp1) * float(ROTATION_DEG) / float(ROTATION_MAX); // milos, current wheel angle
        ROTATION_DEG = temp; // milos, update degrees of rotation
        ROTATION_MAX = int32_t(float(CPR) / 360.0 * float(temp)); // milos, updated
        ROTATION_MID = ROTATION_MAX / 2; // milos, updated
        //brWheelFFB.offset = 0; //milos
        temp1 = int32_t(wheelAngle * float(ROTATION_MAX) / float(ROTATION_DEG)); // milos, here we recover the old wheel angle
        myEnc.Write(ROTATION_MID + temp1 - brWheelFFB.offset); // milos
        //CONFIG_SERIAL.print("Rotation [30-1800]deg: "); // milos
        //CONFIG_SERIAL.println(temp);  // milos
        CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_ROTATION_DEG, temp);//milos, update EEPROM
        break;
      case 'E': //milos, added - turn desktop effects on/off
        ffb_temp = CONFIG_SERIAL.parseInt();
        ffb_temp = constrain(ffb_temp, 0, 31);
        //CONFIG_SERIAL.print("Desktop effects byte: "); //milos, autocentering spring, damper, inertia, friction and ffb out
        for (uint8_t i = 0; i <= 4; i++) { //milos, decode incomming number into individual switches
          bitWrite(effstate, i, bitRead(ffb_temp, i));
        }
        CONFIG_SERIAL.println(effstate, BIN);
        //CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_DSK_EFFC, effstate);//milos, update EEPROM
        break;
      case 'W': //milos, added - configure PWM settings and frequency
        ffb_temp = CONFIG_SERIAL.parseInt();
        ffb_temp = constrain(ffb_temp, 0, 255);
        //CONFIG_SERIAL.print("PWM settings byte: "); //milos, dir enabled, phase correct and pwm frequency
        for (uint8_t i = 0; i < 8; i++) { //milos, decode incomming number into individual bits
          bitWrite(pwmstate, i, bitRead(ffb_temp, i));
        }
#ifdef USE_EEPROM
        SetParam(PARAM_ADDR_PWM_SET, pwmstate);//milos, update EEPROM with new pwm settings
        temp = calcTOP(pwmstate) * minTorquePP; //milos, recalculate new min torque for curent min torque %
        SetParam(PARAM_ADDR_MIN_TORQ, temp);//milos, update min torque in EEPROM
#endif
        CONFIG_SERIAL.println(calcTOP(pwmstate));
        /*for (uint8_t i = 0; i < 8; i++) { //milos, decode incomming number into individual bits
          CONFIG_SERIAL.print(bitRead(pwmstate, 7-i),BIN);
          }
          CONFIG_SERIAL.println("");*/
        //CONFIG_SERIAL.println(1);
        break;
      case 'H': // milos, added - configure the XY shifter calibration
#ifdef USE_XY_SHIFTER
        c = toUpper(CONFIG_SERIAL.read());
        switch (c) {
          case 'A':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 1023);
            sCal[0] = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'B':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 1023);
            sCal[1] = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'C':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 1023);
            sCal[2] = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'D':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 1023);
            sCal[3] = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'E':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 1023);
            sCal[4] = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'F':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 255);
            sConfig = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'G':
            CONFIG_SERIAL.print(sCal[0]);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(sCal[1]);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(sCal[2]);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(sCal[3]);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(sCal[4]);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.println(sConfig);
            break;
          case 'R':
            CONFIG_SERIAL.print(shifterX);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.println(shifterY);
            break;
        }
#else
        CONFIG_SERIAL.println(0);
#endif
        break;
      case 'Y': // milos, added - configure manual calibration for pedals
#ifndef USE_AUTOCALIB
        c = toUpper(CONFIG_SERIAL.read());
        switch (c) {
          case 'A':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            brakeMin = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'B':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            brakeMax = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'C':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            accelMin = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'D':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            accelMax = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'E':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            clutchMin = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'F':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            clutchMax = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'G':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            hbrakeMin = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'H':
            temp = CONFIG_SERIAL.parseInt();
            temp = constrain(temp, 0, 4095);
            hbrakeMax = temp;
            CONFIG_SERIAL.println(1);
            break;
          case 'R':
            CONFIG_SERIAL.print(brakeMin);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(brakeMax);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(accelMin);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(accelMax);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(clutchMin);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(clutchMax);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.print(hbrakeMin);
            CONFIG_SERIAL.print(" ");
            CONFIG_SERIAL.println(hbrakeMax);
            break;
        }
#else
        CONFIG_SERIAL.println(0);
#endif
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
      case 'A': //milos, save all ffb parameters in EEPROM
#ifdef USE_EEPROM
        SaveEEPROMConfig ();
        //CONFIG_SERIAL.println("ok");
        CONFIG_SERIAL.println(1);
#else
        //CONFIG_SERIAL.println("na");
        CONFIG_SERIAL.println(0);
#endif
        break;
      case 'F':
        c = toUpper(CONFIG_SERIAL.read());
        switch (c) {
          case 'G':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configGeneralGain = constrain(ffb_temp, 0, 200);
            //CONFIG_SERIAL.print("General gain [0-200]%: "); // milos
            //CONFIG_SERIAL.println(configGeneralGain);  // milos
            CONFIG_SERIAL.println(1);
            break;
          case 'C':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configConstantGain = constrain(ffb_temp, 0, 200);
            //CONFIG_SERIAL.print("Constant gain [0-200]%: "); // milos
            //CONFIG_SERIAL.println(configConstantGain);  // milos
            CONFIG_SERIAL.println(1);
            break;
          case 'D':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configDamperGain = constrain(ffb_temp, 0, 200);
            //CONFIG_SERIAL.print("Damper gain [0-200]%: ");
            //CONFIG_SERIAL.println(configDamperGain);  // milos
            CONFIG_SERIAL.println(1);
            break;
          case 'F':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configFrictionGain = constrain(ffb_temp, 0, 200);
            //CONFIG_SERIAL.print("Friction gain [0-200]%: "); // milos
            //CONFIG_SERIAL.println(configFrictionGain);  // milos
            CONFIG_SERIAL.println(1);
            break;
          case 'S':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configPeriodicGain = constrain(ffb_temp, 0, 200);
            //CONFIG_SERIAL.print("Periodic gain [0-200]%: "); // milos
            //CONFIG_SERIAL.println(configPeriodicGain);  // milos
            CONFIG_SERIAL.println(1);
            break;
          case 'M':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configSpringGain = constrain(ffb_temp, 0, 200);
            //CONFIG_SERIAL.print("Spring gain [0-200]%: "); // milos
            //CONFIG_SERIAL.println(configSpringGain);  // milos
            CONFIG_SERIAL.println(1);
            break;
          case 'I':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configInertiaGain = constrain(ffb_temp, 0, 200);
            //CONFIG_SERIAL.print("Inertia gain [0-200]%: "); // milos
            //CONFIG_SERIAL.println(configInertiaGain);  // milos
            CONFIG_SERIAL.println(1);
            break;
          case 'A':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configCenterGain = constrain(ffb_temp, 0, 200);
            //CONFIG_SERIAL.print("Center gain [0-200]%: "); // milos
            //CONFIG_SERIAL.println(configCenterGain);  // milos
            CONFIG_SERIAL.println(1);
            break;
          case 'B':
            ffb_temp = CONFIG_SERIAL.parseInt();
            configStopGain = constrain(ffb_temp, 0, 200);
            //CONFIG_SERIAL.print("Stop gain [0-200]%: ");  // milos
            //CONFIG_SERIAL.println(configStopGain);  // milos
            CONFIG_SERIAL.println(1);
            break;
          case 'J':
            ffb_temp = CONFIG_SERIAL.parseInt();
            ffb_temp = constrain(ffb_temp, 0, 200); //milos
            minTorquePP = (f32)ffb_temp * 0.001; // milos, max is 20% or 0.2
            MM_MIN_MOTOR_TORQUE = (u16)(minTorquePP * (f32)MM_MAX_MOTOR_TORQUE); //milos
            //CONFIG_SERIAL.print("Min torque PWM [0-200]*0.1%: ");  // milos
            //CONFIG_SERIAL.println(MM_MIN_MOTOR_TORQUE);  // milos
            CONFIG_SERIAL.println(1);
            break;
            /*case 'K': //milos, commented, once set at compile time this must not be changed anymore
              temp = CONFIG_SERIAL.parseInt();
              if (temp > MM_MIN_MOTOR_TORQUE)
              {
                MM_MAX_MOTOR_TORQUE = constrain(temp, 1, TOP);
              }
              //CONFIG_SERIAL.print("Max torque: ");  // milos
              CONFIG_SERIAL.println(temp);  // milos
              break;*/
        }
        break;
    }
  }
}
