// milos, completely rewritten - all possible configurations of PWM and DAC settings are handeled here

#include "Config.h"
#include <digitalWriteFast.h>

void InitPWM() {

  pinModeFast(DIR_PIN, OUTPUT);
  pinModeFast(FFBCLIP_LED_PIN, OUTPUT); //milos
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
#else
  //pinModeFast(PWM_PIN_L, OUTPUT);
  //pinModeFast(PWM_PIN_R, OUTPUT);

#if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)

  /*if (bitRead(pwmstate, 0)) { // if phase correct PWM mode (pwmstate bit0=1)

    // Timer 1 configuration: prescaler: clockI/O / 1
    // outputs enabled, phase-correct PWM, top of 400 or 1000
    // PWM frequency calculation : 16MHz / 1 (prescaler) / 2 (phase-correct) / 1000 (top) = 8 kHz
    // PWM frequency calculation : 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20 kHz
    TCCR1A = 0b10100000;
    TCCR1B = 0b00010001;//0b00010001;
    ICR1 = TOP;*/

  //} else { // if fast PWM mode (pwmstate bit0=0)

  PWM16Begin(); // Timer1 configuration: fast pwm mode or phase correct, depending on pwmstate byte
  // On the Arduino UNO T1A is Pin 9 and T1B is Pin 10
  PWM16A(0);  // Set initial PWM value for Pin 9
  PWM16EnableA();  // Turn PWM on for Pin 9
  PWM16B(0);  // Set initial PWM value for Pin 10
  PWM16EnableB();  // Turn PWM on for Pin 10
  // }
#endif // of which ATmega
#endif // of USE_MCP4275

  for (uint8_t i = 0; i < 3; i++) { // milos, added - blink FFB clip LED few times at startup
    digitalWriteFast(FFBCLIP_LED_PIN, HIGH);
    delay(20);
    digitalWriteFast(FFBCLIP_LED_PIN, LOW);
    delay(20);
  }
}

void SetPWM (s32 torque)  { // torque between -MM_MAX_MOTOR and +MM_MAX_MOTOR

  //Serial.println(TCCR1A, BIN);

  //milos, added - turn on FFB clip LED if max FFB signal reached (shows 90-99% of FFB signal as linear increase from 0 to 1/4 of full brightness)
  float level = 0.01 * configGeneralGain;
  if (abs(torque) >= 0.9 * MM_MAX_MOTOR_TORQUE * level && abs(torque) < level * MM_MAX_MOTOR_TORQUE - 1) {
    analogWrite(FFBCLIP_LED_PIN, map(abs(torque), 0.9 * MM_MAX_MOTOR_TORQUE * level, level * MM_MAX_MOTOR_TORQUE, 1, 63));
  } else if (abs(torque) >= level * MM_MAX_MOTOR_TORQUE - 1) {
    analogWrite(FFBCLIP_LED_PIN, 255); // for 100% FFB show full brightness
  } else {
    analogWrite(FFBCLIP_LED_PIN, 0); // if under 90% FFB turn off LED
  }

#ifndef USE_LOAD_CELL // milos, only allow FFB balance if not using load cell
  FFB_bal = (f32)(LC_scaling - 128) / 255.0;
  if (FFB_bal > 0) {
    L_bal = 1.0 - FFB_bal;
    R_bal = 1.0;
  } else if (FFB_bal < 0) {
    L_bal = 1.0;
    R_bal = 1.0 + FFB_bal;
  } else {
    L_bal = 1.0;
    R_bal = 1.0;
  }
#else // otherwise just set both at max
  L_bal = 1.0;
  R_bal = 1.0;
#endif

#ifdef USE_MCP4725 //milos, added - FFB signal as analog external DAC output (uses 2x MCP4725 i2C 12bit chips)
  if (bitRead(pwmstate, 7)) { // if DAC out enabled (pwmstate bit7=1)
    if (!bitRead(pwmstate, 6)) { // if DAC+- mode enabled (pwmstate bit6=0)
      if (torque > 0) {
        dac0.setVoltage(0, false, 0); // left force is 0
        dac1.setVoltage(map(torque, 0, MM_MAX_MOTOR_TORQUE, minTorquePP * MM_MAX_MOTOR_TORQUE, R_bal * MM_MAX_MOTOR_TORQUE), false, 0); // right force is scaled from min to max
      } else if (torque < 0) {
        dac0.setVoltage(map(-torque, 0, MM_MAX_MOTOR_TORQUE, minTorquePP * MM_MAX_MOTOR_TORQUE, L_bal * MM_MAX_MOTOR_TORQUE), false, 0); // left force is scaled from min to max
        dac1.setVoltage(0, false, 0); // right force is 0
      } else {
        dac0.setVoltage(0, false, 0);
        dac1.setVoltage(0, false, 0);
      }
    } else { // if DAC+dir enabled (pwmstate bit6=1)
      if (torque >= 0) {
        digitalWriteFast(DIR_PIN, HIGH);
      } else {
        digitalWriteFast(DIR_PIN, LOW);
      }
      torque = map (abs(torque), 0, MM_MAX_MOTOR_TORQUE, minTorquePP * MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
      dac0.setVoltage(torque, false, 0); // update left DAC
      dac1.setVoltage(0, false, 0); // keep right DAC at zero
    }
  }
#else // milos, otherwise output FFB as PWM signals

  /*if (bitRead(pwmstate, 0)) { // if phase correct PWM mode (pwmstate bit0=1)
    if (!bitRead(pwmstate, 1)) { // if PWM+- mode (pwmstate bit1=0)
      if (!bitRead(pwmstate, 6)) { // if pwmstate bit1=0 and bit6=0
        // milos, new variant of the original just with PWM+- mode
        if (torque > 0) {
          torque = map (torque, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, R_bal * MM_MAX_MOTOR_TORQUE);
          OCR1A = 0;
          OCR1B = torque;
        } else if (torque < 0) {
          torque = map (-torque, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, L_bal * MM_MAX_MOTOR_TORQUE);
          OCR1A = torque;
          OCR1B = 0;
        } else {
          OCR1A = 0;
          OCR1B = 0;
        }
      } else { // if PWM0.50.100 mode (pwmstate bit1=0 and bit6=1)
        if (torque > 0) {
          torque = map (torque, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 + MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
          OCR1A = torque;
          OCR1B = 0;
        } else if (torque < 0) {
          torque = map (-torque, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 - MM_MIN_MOTOR_TORQUE, 0);
          OCR1A = torque;
          OCR1B = 0;
        } else {
          OCR1A = MM_MAX_MOTOR_TORQUE / 2;
          OCR1B = 0;
        }
      }
    } else if (bitRead(pwmstate, 1)) { // if pwmstate bit1=1
      if (!bitRead(pwmstate, 6)) { // if PWM+dir mode (pwmstate bit1=1, bit6=0)
        if (torque >= 0) {
          digitalWriteFast(DIR_PIN, HIGH);
        } else {
          digitalWriteFast(DIR_PIN, LOW);
        }
        torque = map (abs(torque), 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
        OCR1A = torque;
        OCR1B = 0; // milos, this becomes unused
      } else { // if RCM mode (pwmstate bit1=1, bit6=1)
        if (torque > 0) {
          torque = map (torque, 0, MM_MAX_MOTOR_TORQUE, RCMscaler(RCM_zer) * (MM_MAX_MOTOR_TORQUE + MM_MIN_MOTOR_TORQUE), RCMscaler(RCM_max) * MM_MAX_MOTOR_TORQUE);
          OCR1A = torque;
          OCR1B = 0;
          } else if (torque < 0) {
          torque = map (-torque, 0, MM_MAX_MOTOR_TORQUE, RCMscaler(RCM_zer) * (MM_MAX_MOTOR_TORQUE - MM_MIN_MOTOR_TORQUE), RCMscaler(RCM_min) * MM_MAX_MOTOR_TORQUE);
          OCR1A = torque;
          OCR1B = 0;
          } else {
          OCR1A = RCMscaler(RCM_zer) * MM_MAX_MOTOR_TORQUE;
          OCR1B = 0;
          }
      }
    }

    } else {*/ // if fast top PWM mode (pwmstate bit0=0)

  if (!bitRead(pwmstate, 1)) { // if pwmstate bit1=0
    if (!bitRead(pwmstate, 6)) { // if PWM+- mode (pwmstate bit1=0, bit6=0)
      if (torque > 0) {
        torque = map (torque, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, R_bal * MM_MAX_MOTOR_TORQUE);
        PWM16A(0);
        PWM16B(torque);
      } else if (torque < 0) {
        torque = map (-torque, 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, L_bal * MM_MAX_MOTOR_TORQUE);
        PWM16A(torque);
        PWM16B(0);
      } else {
        PWM16A(0);
        PWM16B(0);
      }
    } else { // if PWM0.50.100 mode (pwmstate bit1=0 and bit6=1)
      if (torque > 0) {
        torque = map (torque, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 + MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
        PWM16A(torque);
        PWM16B(0);
      } else if (torque < 0) {
        torque = map (-torque, 0, MM_MAX_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE / 2 - MM_MIN_MOTOR_TORQUE, 0);
        PWM16A(torque);
        PWM16B(0);
      } else {
        PWM16A(MM_MAX_MOTOR_TORQUE / 2);
        PWM16B(0);
      }
    }
  } else if (bitRead(pwmstate, 1)) { // if pwmstate bit1=1
    if (!bitRead(pwmstate, 6)) { // if PWM+dir mode (pwmstate bit1=1, bit6=0)
      if (torque >= 0) {
        digitalWriteFast(DIR_PIN, HIGH);
      } else {
        digitalWriteFast(DIR_PIN, LOW);
      }
      torque = map (abs(torque), 0, MM_MAX_MOTOR_TORQUE, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE);
      PWM16A(torque);
      PWM16B(0);
    } else { // if RCM mode (pwmstate bit1=1, bit6=1)
      if (torque > 0) {
        torque = map (torque, 0, MM_MAX_MOTOR_TORQUE, RCM_zer * (1.0 + minTorquePP), RCM_max);
        //Serial.println(torque);
        PWM16A(torque);
        PWM16B(0);
      } else if (torque < 0) {
        torque = map (-torque, 0, MM_MAX_MOTOR_TORQUE, RCM_zer * (1.0 - minTorquePP), RCM_min);
        //Serial.println(torque);
        PWM16A(torque);
        PWM16B(0);
      } else {
        //PWM16A(RCMscaler(RCM_zer) * MM_MAX_MOTOR_TORQUE);
        PWM16A(RCM_zer);
        PWM16B(0);
      }
    }
  }
#endif
}

#ifndef USE_MCP4725
void PWM16Begin() { // milos - added, reconfigure timer1 automatically depending on pwm settings
  // Stop Timer/Counter1
  TCCR1A = 0; // Timer/Counter1 Control Register A
  TCCR1B = 0; // Timer/Counter1 Control Register B
  TIMSK1 = 0; // Timer/Counter1 Interrupt Mask Register
  TIFR1 = 0;  // Timer/Counter1 Interrupt Flag Register
  ICR1 = TOP; // milos, set upper counter flag
  OCR1A = 0;  // Default to 0% PWM
  OCR1B = 0;  // Default to 0% PWM

  if (bitRead(pwmstate, 1) && bitRead(pwmstate, 6)) { // if RMC pwm mode
    TCCR1B |= (1 << CS11); // milos, Set clock prescaler to 8
  } else { // for all other pwm modes
    TCCR1B |= (1 << CS10); // Set clock prescaler to 1 for maximum PWM frequency
  }
  if (bitRead(pwmstate, 0)) { // if pwmstate bit0=1, configure timer for phase correct mode
    // milos, Set Timer/Counter1 to Waveform Generation Mode 8: Phase and Frequency Correct PWM with TOP set by ICR1
    TCCR1B |= (1 << WGM13);
  } else {  // if pwmstate bit0=0, fast pwm mode
    // milos, Set Timer/Counter1 to Waveform Generation Mode 14: Fast PWM with TOP set by ICR1
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);
  }
}

void PWM16EnableA() {  // milos
  // Enable Fast PWM on Pin 9: Set OC1A at BOTTOM and clear OC1A on OCR1A compare
  TCCR1A |= (1 << COM1A1);
  pinModeFast(PWM_PIN_L, OUTPUT);

  // milos, added - set minimal values right away (pwm0.50.100 and rcm have a non-zero duty cycle for zero force)
  /*if (!bitRead(pwmstate, 1) && bitRead(pwmstate, 6)) { // milos, pwmstate bit1=0 and bit6=1 - if we have set pwm0.50.100 mode
    OCR1A = TOP / 2; // Default to 50% PWM
    } else if (bitRead(pwmstate, 1) && bitRead(pwmstate, 6)) { // milos, pwmstate bit1=1 and bit6=1 - if we have set RCM mode
    OCR1A = RCM_zer; // Default to zero force RCM duty cycle value (1.5ms pulse width)
    } else {
    OCR1A = 0;   // Default to 0% PWM
    }*/
}

void PWM16EnableB() {  // milos
  // Enable Fast PWM on Pin 10: Set OC1B at BOTTOM and clear OC1B on OCR1B compare
  TCCR1A |= (1 << COM1B1);
  pinModeFast(PWM_PIN_R, OUTPUT);
}

inline void PWM16A(uint16_t PWMValue) { // milos
  OCR1A = constrain(PWMValue, 0, TOP);
}

inline void PWM16B(uint16_t PWMValue) { // milos
  OCR1B = constrain(PWMValue, 0, TOP);
}
#endif
