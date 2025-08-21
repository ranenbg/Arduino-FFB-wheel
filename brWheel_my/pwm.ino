// milos, completely rewritten - all possible configurations of PWM and DAC settings are handeled here

#include "Config.h"
#include <digitalWriteFast.h>

void InitPWM() {
  pinModeFast(DIR_PIN, OUTPUT);
  TOP = calcTOP(pwmstate); // milos, this will set appropriate TOP value for all PWM modes, depending on pwmstate loaded from EEPROM
  MM_MAX_MOTOR_TORQUE = TOP;
  minTorquePP = ((f32)MM_MIN_MOTOR_TORQUE) / ((f32)MM_MAX_MOTOR_TORQUE); // milos
  RCM_min *= RCMscaler(pwmstate); // milos - takes into account fast pwm or phase correct mode
  RCM_zer *= RCMscaler(pwmstate); // milos
  RCM_max *= RCMscaler(pwmstate); // milos
#ifdef USE_MCP4725 //milos, added
  dac0.begin(0x60); // initialize dac0
  dac1.begin(0x61); // initialize dac1
  dac0.setVoltage(0, true, 0); // set voltage on dac0 (save voltage after power down)
  dac1.setVoltage(0, true, 0); // set voltage on dac1 (save voltage after power down)
#ifdef USE_TWOFFBAXIS
  pinModeFast(PWM_PIN_R, OUTPUT); // milos, dir pin at D10 for 2nd DAC channel in DAC+dir mode
#endif // end of 2 ffb axis
#else // milos, for pwm output
#if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)
  PWM16Begin(); // Timer1 and Timer3 configuration: frequency and mode depend on pwmstate byte
  PWM16A(0);  // Set initial PWM value for Pin 9
  PWM16EnableA();  // Turn PWM on for Pin 9
  PWM16B(0);  // Set initial PWM value for Pin 10
  PWM16EnableB();  // Turn PWM on for Pin 10
#ifdef USE_TWOFFBAXIS
  PWM16C(0);  // Set initial PWM value for Pin 11
  PWM16EnableC();  // Turn PWM on for Pin 11
  PWM16D(0);  // Set initial PWM value for Pin 5
  PWM16EnableD();  // Turn PWM on for Pin 5
#endif // end of 2 ffb axis
#endif // of which ATmega
#endif // of USE_MCP4275

#ifndef USE_PROMICRO
  pinMode(FFBCLIP_LED_PIN, OUTPUT); // milos, if no proMicro we can use ffb clip led
  blinkFFBclipLED(); // milos, signals end of configuration
#else // for proMicro
//#ifndef USE_CENTERBTN  // milos, we can only use it if no center button using pin 3 (for proMicro center button is on pin 2, so we skip this check)
#ifndef USE_MCP4725 // milos, we can only use it if DAC is not using i2C pins 2,3
#ifndef USE_ADS1015 // milos, we can only use it if ADS1015 is not using i2C pins 2,3
#ifndef USE_AS5600 // milos, we can only use it if AS5600 is not using i2C pin 2,3
  pinMode(FFBCLIP_LED_PIN, OUTPUT); // milos, for promicro we can only use ffb clip led on D3 if not using all above
  blinkFFBclipLED(); // milos, signals end of configuration
#endif // end of as5600
#endif // end of ads1015
#endif // end of mcp4725
//#endif // end of center button
#endif // end of proMicro
}

void blinkFFBclipLED() { // milos, added - blink FFB clip LED a few times at startup to indicate succesful boot
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(FFBCLIP_LED_PIN, HIGH);
    delay(20);
    digitalWrite(FFBCLIP_LED_PIN, LOW);
    delay(20);
  }
}

void activateFFBclipLED(s32 t) {  // milos, added - turn on FFB clip LED if max FFB signal reached (shows 90-99% of FFB signal as linear increase from 0 to 1/4 of full brightness)
  float level = 0.01 * configGeneralGain;
  if (abs(t) >= 0.9 * MM_MAX_MOTOR_TORQUE * level && abs(t) < level * MM_MAX_MOTOR_TORQUE - 1) {
    analogWrite(FFBCLIP_LED_PIN, map(abs(t), 0.9 * MM_MAX_MOTOR_TORQUE * level, level * MM_MAX_MOTOR_TORQUE, 1, 63)); // for 90%-99% ffb map brightness linearly from 1-63 (out of 255)
  } else if (abs(t) >= level * MM_MAX_MOTOR_TORQUE - 1) {
    digitalWrite(FFBCLIP_LED_PIN, HIGH); // for 100% FFB set full brightness
  } else {
    digitalWrite(FFBCLIP_LED_PIN, LOW); // if under 90% FFB turn off LED
  }
}

//void SetPWM (s32 torque)  { //torque between -MM_MAX_MOTOR and +MM_MAX_MOTOR // milos, torque is xFFB, while yFFB is passed from torqueY global variable outside of this function
void SetPWM (s32v *torque) { // milos, takes pointer struct as argument - 2 axis FFB data
  if (torque != NULL) { // milos, this check is always required for pointers
#ifndef USE_PROMICRO
    activateFFBclipLED(torque->x); // milos, if no promicro we can use ffb clip led on D13
#else // for proMicro
//#ifndef USE_CENTERBTN // milos, we can only use it if no center button using pin 3 (for proMicro center button is on pin 2, so we skip this check)
#ifndef USE_MCP4725 // milos, we can only use it if DAC is not using i2C pins 2,3
#ifndef USE_ADS1015 // milos, we can only use it if ADS1015 is not using i2C pins 2,3
#ifndef USE_AS5600 // milos, we can only use it if AS5600 is not using i2C pin 2,3
    activateFFBclipLED(torque->x); // milos, for promicro we can only use ffb clip led on D3 if not using all above
#endif // end of as5600
#endif // end of ads1015
#endif // end of mcp4725
//#endif // end of center button
#endif // end of proMicro

#ifndef USE_LOAD_CELL // milos, only allow FFB balance if not using load cell
    FFB_bal = (f32)(LC_scaling - 128) / 255.0; // milos, max value is 0.5
    if (FFB_bal >= 0) {
      L_bal = 1.0 - FFB_bal;
      R_bal = 1.0;
    } else if (FFB_bal < 0) {
      L_bal = 1.0;
      R_bal = 1.0 + FFB_bal;
    }/* else {
      L_bal = 1.0;
      R_bal = 1.0;
    }*/
#else // otherwise just set both at max
    L_bal = 1.0;
    R_bal = 1.0;
#endif // end of load cell

#ifdef USE_MCP4725 //milos, added - FFB signal as analog external DAC output (uses 2x MCP4725 i2C 12bit chips)
    if (bitRead(pwmstate, 7)) { // if DAC out enabled (pwmstate bit7=1)
      if (!bitRead(pwmstate, 6)) { // if DAC+- mode enabled (pwmstate bit6=0)
        if (!bitRead(pwmstate, 5)) { // if (pwmstate bit5=0)
          // milos, if we use 1 or 2 FFB axis we have dac+- mode (for 1 ffb axis it's gona be only on xFFB)
          if (torque->x > 0) {
            dac0.setVoltage(0, false, 0); // left force is 0
            dac1.setVoltage(map(torque->x, 0, MM_MAX_MOTOR_TORQUE, minTorquePP * MM_MAX_MOTOR_TORQUE, R_bal * MM_MAX_MOTOR_TORQUE), false, 0); // right force is scaled from min to max
          } else if (torque->x < 0) {
            dac0.setVoltage(map(-torque->x, 0, MM_MAX_MOTOR_TORQUE, minTorquePP * MM_MAX_MOTOR_TORQUE, L_bal * MM_MAX_MOTOR_TORQUE), false, 0); // left force is scaled from min to max
            dac1.setVoltage(0, false, 0); // right force is 0
          } else {
            dac0.setVoltage(0, false, 0);
            dac1.setVoltage(0, false, 0);
          }
        } else { // milos, if DAC0.50.100 mode (pwmstate bit5=1, bit6=0)
          if (torque->x > 0) {
            torque->x = map(torque->x, 0, MM_MAX_MOTOR_TORQUE, (MM_MAX_MOTOR_TORQUE >> 1) + MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
            dac0.setVoltage(torque->x, false, 0);
          } else if (torque->x < 0) {
            torque->x = map(-torque->x, 0, MM_MAX_MOTOR_TORQUE, (MM_MAX_MOTOR_TORQUE >> 1) - MM_MIN_MOTOR_TORQUE, 0);
            dac0.setVoltage(torque->x, false, 0);
          } else {
            dac0.setVoltage(MM_MAX_MOTOR_TORQUE / 2, false, 0);
          }
#ifndef USE_TWOFFBAXIS // milos, if we use 1 FFB axis for dac0-50-100 mode
          dac1.setVoltage(MM_MAX_MOTOR_TORQUE >> 1, false, 0); // milos, set 2nd dac at half range
#else // if 2 ffb axis and mcp4725
          if (torque->y > 0) {
            torque->y = map(torque->y, 0, MM_MAX_MOTOR_TORQUE, (MM_MAX_MOTOR_TORQUE >> 1) + MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
            dac1.setVoltage(torque->y, false, 0);
          } else if (torque->y < 0) {
            torque->y = map(-torque->y, 0, MM_MAX_MOTOR_TORQUE, (MM_MAX_MOTOR_TORQUE >> 1) - MM_MIN_MOTOR_TORQUE, 0);
            dac1.setVoltage(torque->y, false, 0);
          } else {
            dac1.setVoltage(MM_MAX_MOTOR_TORQUE / 2, false, 0);
          }
#endif // end of 2 ffb axis
        }
      } else { // if DAC+DIR enabled (pwmstate bit6=1)
        if (torque->x >= 0) {
          digitalWriteFast(DIR_PIN, HIGH);
        } else {
          digitalWriteFast(DIR_PIN, LOW);
        }
        torque->x = map(abs(torque->x), 0, MM_MAX_MOTOR_TORQUE, minTorquePP * MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
        dac0.setVoltage(torque->x, false, 0); // update 1st DAC
#ifndef USE_TWOFFBAXIS // milos, if we use 1 FFB axis
        dac1.setVoltage(0, false, 0); // keep 2nd DAC at zero
#else // if 2 ffb axis and mcp4725
        // milos, 2CH DAC+DIR mode
        if (torque->y >= 0) {
          digitalWriteFast(PWM_PIN_R, HIGH); // milos, positive force (up)
        } else {
          digitalWriteFast(PWM_PIN_R, LOW); // milos, negative force (down)
        }
        torque->y = map(abs(torque->y), 0, MM_MAX_MOTOR_TORQUE, minTorquePP * MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
        dac1.setVoltage(torque->y, false, 0); // update 2nd DAC
#endif // end of 2 ffb axis
      }
    } else { // milos, if bit7 of pwmstate is LOW, disable dac output
      int16_t dacZeroOut = 0;
      if (!bitRead(pwmstate, 6) && bitRead(pwmstate, 5)) dacZeroOut = MM_MAX_MOTOR_TORQUE >> 1; // milos, for dac0-50.100 mode, zero output is at half range
      dac0.setVoltage(dacZeroOut, false, 0); // zero 1st DAC
      dac1.setVoltage(dacZeroOut, false, 0); // zero 2nd DAC
    }
#else // milos, no mcp4725 - output FFB as PWM signals
#ifndef USE_TWOFFBAXIS // milos, if we use 1 FFB axis
    if (!bitRead(pwmstate, 1)) { // if pwmstate bit1=0
      if (!bitRead(pwmstate, 6)) { // if PWM+- mode (pwmstate bit1=0, bit6=0)
        if (torque->x > 0) {
          torque->x = map(torque->x, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, R_bal * MM_MAX_MOTOR_TORQUE);
          PWM16A(0);
          PWM16B(torque->x);
          digitalWriteFast(DIR_PIN, HIGH); //use dir pin as BTS7960 pwm motor enable signal
        } else if (torque->x < 0) {
          torque->x = map(-torque->x, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, L_bal * MM_MAX_MOTOR_TORQUE);
          PWM16A(torque->x);
          PWM16B(0);
          digitalWriteFast(DIR_PIN, HIGH);
        } else {
          PWM16A(0);
          PWM16B(0);
          digitalWriteFast(DIR_PIN, LOW); // disable bts output when no pwm signal to make it rotate freely
        }
      } else { // if PWM0.50.100 mode (pwmstate bit1=0 and bit6=1)
        if (torque->x > 0) {
          torque->x = map(torque->x, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 + MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
          PWM16A(torque->x);
        } else if (torque->x < 0) {
          torque->x = map(-torque->x, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 - MM_MIN_MOTOR_TORQUE, 0);
          PWM16A(torque->x);
        } else {
          PWM16A(MM_MAX_MOTOR_TORQUE / 2);
        }
        PWM16B(0);
      }
    } else if (bitRead(pwmstate, 1)) { // if pwmstate bit1=1
      if (!bitRead(pwmstate, 6)) { // if PWM+dir mode (pwmstate bit1=1, bit6=0)
        if (torque->x >= 0) {
          digitalWriteFast(DIR_PIN, HIGH);
        } else {
          digitalWriteFast(DIR_PIN, LOW);
        }
        torque->x = map(abs(torque->x), 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
        PWM16A(torque->x);
      } else { // if RCM mode (pwmstate bit1=1, bit6=1)
        if (torque->x > 0) {
          torque->x = map(torque->x, 0, MM_MAX_MOTOR_TORQUE, RCM_zer * (1.0 + minTorquePP), RCM_max);
          PWM16A(torque->x);
        } else if (torque->x < 0) {
          torque->x = map(-torque->x, 0, MM_MAX_MOTOR_TORQUE, RCM_zer * (1.0 - minTorquePP), RCM_min);
          PWM16A(torque->x);
        } else {
          PWM16A(RCM_zer);
        }
      }
      PWM16B(RCM_zer);
      //digitalWriteFast(PWM_PIN_R, LOW);
    }
#else // milos, if we use 2 FFB axis
    if (!bitRead(pwmstate, 1)) { // if pwmstate bit1=0
      if (!bitRead(pwmstate, 6)) { // if 2CH PWM+- mode (pwmstate bit1=0, bit6=0)
        if (torque->x > 0) { // milos, X axis FFB, pwm+-, D9 (left), D10 (right)
          torque->x = map(torque->x, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
          PWM16A(0);
          PWM16B(torque->x);
        } else if (torque->x < 0) {
          torque->x = map(-torque->x, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
          PWM16A(torque->x);
          PWM16B(0);
        } else {
          PWM16A(0);
          PWM16B(0);
        }
        if (torque->y >= 0) {  // milos, Y axis FFB, pwm+-, D11 (up), D5 (down)
          torque->y = map(torque->y, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
          PWM16C(torque->y);
          PWM16D(0);
        } else if (torque->y < 0) {
          torque->y = map(-torque->y, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
          PWM16C(0);
          PWM16D(torque->y);
        } else {
          PWM16C(0);
          PWM16D(0);
        }
      } else {  // milos, 2CH PWM0.50.100 mode (pwmstate bit1=0, bit6=1)
#ifndef USE_TCA9548 // milos, only available if not using 2 mag encoders via i2C multilexer
        //PWM16C(0);
        //PWM16D(0);
        //digitalWriteFast(PWM_PIN_D, LOW);
        //digitalWriteFast(PWM_PIN_U, LOW);
        if (torque->x > 0) {
          torque->x = map(torque->x, 0, MM_MAX_MOTOR_TORQUE, (MM_MAX_MOTOR_TORQUE >> 1) + MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
          PWM16A(torque->x);
        } else if (torque->x < 0) {
          torque->x = map(-torque->x, 0, MM_MAX_MOTOR_TORQUE, (MM_MAX_MOTOR_TORQUE >> 1) - MM_MIN_MOTOR_TORQUE, 0);
          PWM16A(torque->x);
        } else {
          PWM16A(MM_MAX_MOTOR_TORQUE / 2);
        }
        if (torque->y > 0) {
          torque->y = map(torque->y, 0, MM_MAX_MOTOR_TORQUE, (MM_MAX_MOTOR_TORQUE >> 1) + MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
          PWM16B(torque->y);
        } else if (torque->y < 0) {
          torque->y = map(-torque->y, 0, MM_MAX_MOTOR_TORQUE, (MM_MAX_MOTOR_TORQUE >> 1) - MM_MIN_MOTOR_TORQUE, 0);
          PWM16B(torque->y);
        } else {
          PWM16B(MM_MAX_MOTOR_TORQUE / 2);
        }
#endif // end of tca
      }
    } else { // milos, if pwmstate bit1=1
#ifndef USE_TCA9548 // milos, only available if not using 2 mag encoders via i2C multilexer
      if (!bitRead(pwmstate, 6)) { // milos, if 2CH PWM+DIR mode (pwmstate bit1=1, bit6=0)
        if (torque->x >= 0) {
          digitalWriteFast(PWM_PIN_U, HIGH); // milos, pin D11
        } else {
          digitalWriteFast(PWM_PIN_U, LOW);
        }
        torque->x = map(abs(torque->x), 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
        PWM16A(torque->x); // milos, use pin D9 for xFFB
        if (torque->y >= 0) {
          digitalWriteFast(PWM_PIN_D, HIGH); // milos, pin D5
        } else {
          digitalWriteFast(PWM_PIN_D, LOW);
        }
        torque->y = map(abs(torque->y), 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
        PWM16B(torque->y); // milos, use pin D10 for yFFB
      } else { // milos, if 2CH RCM mode (pwmstate bit1=1, bit6=1)
        // milos, 2CH RCM mode (unfortunately, not enough memory left for it so I commented it out)
        /*if (torque->x > 0) {
          torque->x = map(torque->x, 0, MM_MAX_MOTOR_TORQUE, RCM_zer * (1.0 + minTorquePP), RCM_max);
          PWM16A(torque->x);
          } else if (torque->x < 0) {
          torque->x = map(-torque->x, 0, MM_MAX_MOTOR_TORQUE, RCM_zer * (1.0 - minTorquePP), RCM_min);
          PWM16A(torque->x);
          } else {
          PWM16A(RCM_zer);
          }
          if (torque->y > 0) {
          torque->y = map(torque->y, 0, MM_MAX_MOTOR_TORQUE, RCM_zer * (1.0 + minTorquePP), RCM_max);
          PWM16B(torque->y);
          } else if (torque->y < 0) {
          torque->y = map(-torque->y, 0, MM_MAX_MOTOR_TORQUE, RCM_zer * (1.0 - minTorquePP), RCM_min);
          PWM16B(torque->y);
          } else {
          PWM16B(RCM_zer);
          }*/
        PWM16A(RCM_zer); // milos, for now only zero it out
        PWM16B(RCM_zer);
      }
#endif // end of tca
    }
#endif // end of 2 ffb axis
#endif // end of use mcp4275
  }
}

#ifndef USE_MCP4725
void PWM16Begin() { // milos - added, reconfigure timer1 automatically depending on pwm settings
  // Stop Timer/Counter1
  TCCR1A = 0; // Timer/Counter1 Control Register A
  TCCR1B = 0; // Timer/Counter1 Control Register B
  TIMSK1 = 0; // Timer/Counter1 Interrupt Mask Register
  TIFR1 = 0;  // Timer/Counter1 Interrupt Flag Register
  ICR1 = TOP; // milos, set upper counter flag
  OCR1A = 0;  // Default to 0% PWM, D9
  OCR1B = 0;  // Default to 0% PWM, D10
#ifdef USE_TWOFFBAXIS
  OCR1C = 0;  // Default to 0% PWM, D11
#endif // end of 2 ffb axis

  if (bitRead(pwmstate, 1) && bitRead(pwmstate, 6)) { // if RMC pwm mode
    TCCR1B |= (1 << CS11); // milos, Set clock prescaler to 8
  } else { // for all other pwm modes
    TCCR1B |= (1 << CS10); // Set clock prescaler to 1 for maximum PWM frequency
  }
  if (bitRead(pwmstate, 0)) { // if pwmstate bit0=1, configure timer for phase correct mode
    // milos, Set Timer/Counter1 to Waveform Generation Mode 10: Phase and Frequency Correct PWM with TOP set by ICR1
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13);
  } else {  // if pwmstate bit0=0, fast pwm mode
    // milos, Set Timer/Counter1 to Waveform Generation Mode 14: Fast PWM with TOP set by ICR1
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);
  }
#ifdef USE_TWOFFBAXIS
  // Stop Timer/Counter3
  TCCR3A = 0; // Timer/Counter3 Control Register A
  TCCR3B = 0; // Timer/Counter3 Control Register B
  TIMSK3 = 0; // Timer/Counter3 Interrupt Mask Register
  TIFR3 = 0;  // Timer/Counter3 Interrupt Flag Register
  ICR3 = TOP; // milos, set upper counter flag
  OCR3A = 0;  // Default to 0% PWM, D5

  if (bitRead(pwmstate, 1) && bitRead(pwmstate, 6)) { // if RMC pwm mode
    TCCR3B |= (1 << CS31); // milos, Set clock prescaler to 8
  } else { // for all other pwm modes
    TCCR3B |= (1 << CS30); // Set clock prescaler to 1 for maximum PWM frequency
  }
  if (bitRead(pwmstate, 0)) { // if pwmstate bit0=1, configure timer for phase correct mode
    // milos, Set Timer/Counter3 to Waveform Generation Mode 10: Phase and Frequency Correct PWM with TOP set by ICR3
    TCCR3A |= (1 << WGM31);
    TCCR3B |= (1 << WGM33);
  } else {  // if pwmstate bit0=0, fast pwm mode
    // milos, Set Timer/Counter3 to Waveform Generation Mode 14: Fast PWM with TOP set by ICR3
    TCCR3A |= (1 << WGM31);
    TCCR3B |= (1 << WGM33) | (1 << WGM32);
  }
#endif // end of 2 ffb axis
}

void PWM16EnableA() {  // milos
  // Enable Fast PWM on Pin 9: Set OC1A at BOTTOM and clear OC1A on OCR1A compare
  TCCR1A |= (1 << COM1A1);
  pinModeFast(PWM_PIN_L, OUTPUT);
}

void PWM16EnableB() {  // milos
  // Enable Fast PWM on Pin 10: Set OC1B at BOTTOM and clear OC1B on OCR1B compare
  TCCR1A |= (1 << COM1B1);
  pinModeFast(PWM_PIN_R, OUTPUT);
}

#ifdef USE_TWOFFBAXIS
void PWM16EnableC() {  // milos
  // Enable Fast PWM on Pin 11: Set OC1B at BOTTOM and clear OC1B on OCR1B compare
  TCCR1A |= (1 << COM1C1);
  pinModeFast(PWM_PIN_U, OUTPUT);
}

void PWM16EnableD() {  // milos
  // Enable Fast PWM on Pin 5: Set OC3A at BOTTOM and clear OC3A on OCR3A compare
  TCCR3A |= (1 << COM3A1);
  pinModeFast(PWM_PIN_D, OUTPUT);
}
#endif // end of 2 ffb axis

inline void PWM16A(uint16_t PWMValue) { // milos
  OCR1A = constrain(PWMValue, 0, TOP);
}

inline void PWM16B(uint16_t PWMValue) { // milos
  OCR1B = constrain(PWMValue, 0, TOP);
}
#ifdef USE_TWOFFBAXIS
inline void PWM16C(uint16_t PWMValue) { // milos
  OCR1C = constrain(PWMValue, 0, TOP);
}

inline void PWM16D(uint16_t PWMValue) { // milos
  OCR3A = constrain(PWMValue, 0, TOP);
}
#endif // end of 2 ffb axis
#endif // end of mcp4725
