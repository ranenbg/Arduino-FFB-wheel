/*
  Force Feedback Wheel

  Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
  Copyright 2013  Saku Kekkonen
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

#include "ffb_pro.h"
#include "QuadEncoder.h"
#include <util/delay.h>

//------------------------------------- Defines ----------------------------------------------------------

const s32 INERTIA_COEF = 16; //milos, added (wheel mass)
const s32 DAMPER_COEF = 16; //milos, added (wheel viscosity)
const s32 FRICTION_COEF = 8; //milos, modified (wheel friction)
const s32 SPRING_COEF = 8; //milos, modified (wheel elasticity)

const s32 AUTO_CENTER_SPRING = 512; //milos, modified
//#define AUTO_CENTER_DAMPER    64 //milos, commented
const s32 BOUNDARY_SPRING	=	 32767; //milos, modified
//#define BOUNDARY_FRICTION		32 //milos, commented

//const f32 gSpeedSet = 0.1f;             //milos, commented
//const f32 gSpringSet = 0.05f; // 0.005  //milos, commented

//milos, commented
/*#define CALIBRATOR_INDEX_SEARCH_TORQUE	200
  #define CALIBRATOR_SPD_THRESHOLD		100//8
  #define CALIBRATION_DAMPER				32//250
  #define CALIBRATOR_HOMING_SPEED			5
  #define CALIBRATOR_HOMING_SPRING		128

  #define CALIBRATION_TIMEOUT_SECONDS		20

  #define SEC2FRAMES(m_sec)		(m_sec*CONTROL_FPS)
  #define INDEX_INT_NUM			1*/

//--------------------------------------- Globals --------------------------------------------------------

// milos, all effects can be divided into two groups:
// [1] time dependant effects:
//     [a] ramp
//     [b] periodic effects: sine, triangle, square, sawtoothup, sawtoothdown
//
// [2] time independant effects:
//     [a] postition dependant (X axis): spring (pos), damper (speed), inertia (acceleration), friction (speed)
//     [b] position independant: constant
//

//milos, commented since it was not used
/*f32 const PROGMEM fir_coefs[NB_TAPS] =
  {
  0.070573279,
  0.208085075,
  0.311521993,
  0.275324301,
  0.133483934,
  0.015952111,
  -0.01492828,
  -1.24126E-05,
  };*/

void cSpeedObs::Init () {
  mLastPos = 0;
  mLastSpeed = 0;
  mLastValueValid = false;
  mCurrentLS = 0;
  for (u8 i = 0; i < NB_TAPS; i++)
    mLastSpeeds[i] = 0;
}

f32 cSpeedObs::Update (s32 new_pos) {
  f32 speed = new_pos - mLastPos; //milos, was s32 speed
  mLastPos = new_pos;
  if (mLastValueValid) {
    s32 t = 2;
    // 		s32 avg_speed = (mLastSpeed*(16-t) + speed*t) >> 5;
    // 		mLastSpeed = avg_speed;
    // 		s32 avg_speed = (mLastSpeed + speed) >> 1;
    // 		mLastSpeed = speed;
    mLastSpeeds[mCurrentLS] = speed;
    u8 fls = mCurrentLS;
    f32 avg_speed = 0;
    for (u8 i = 0; i < NB_TAPS; i++) {
      avg_speed += mLastSpeeds[fls]; //* fir_coefs[i];
      if (fls == 0) {
        fls = NB_TAPS - 1;
      } else {
        fls--;
      }
    }
    avg_speed /= NB_TAPS;
    mCurrentLS++;
    if (mCurrentLS >= NB_TAPS)
      mCurrentLS = 0;
    return (avg_speed);
  }
  mLastValueValid = true;
  mLastSpeed = 0;
  return (0);
}

void cAccelObs::Init () { //milos, added
  mLastSpd = 0;
  mLastAccel = 0;
  mLastValueValid = false;
  mCurrentLA = 0;
  for (u8 i = 0; i < NB_TAPS_A; i++) {
    mLastAccels[i] = 0;
  }
}

f32 cAccelObs::Update (f32 new_spd) { //milos, added, was s32 new_spd
  f32 accel = new_spd - mLastSpd; //s32
  mLastSpd = new_spd;
  if (mLastValueValid) {
    mLastAccels[mCurrentLA] = accel;
    u8 fla = mCurrentLA;
    f32 avg_accel = 0;
    for (u8 i = 0; i < NB_TAPS_A; i++) {
      avg_accel += mLastAccels[fla]; // * fir_coefs[i];
      if (fla == 0) {
        fla = NB_TAPS_A - 1;
      } else {
        fla--;
      }
    }
    avg_accel /= NB_TAPS_A;
    mCurrentLA++;
    if (mCurrentLA >= NB_TAPS_A)
      mCurrentLA = 0;
    return (avg_accel);
  }
  mLastValueValid = true;
  mLastAccel = 0;
  return (0);
}

cFFB::cFFB() {
  mAutoCenter = true;
}

//--------------------------------------- Effects --------------------------------------------------------

s32 ConstrainEffect (s32 val) {
  return (constrain(val, -((s32)MM_MAX_MOTOR_TORQUE), (s32)MM_MAX_MOTOR_TORQUE));
}

f32 wDegScl() { // milos, added - scaling factor to convert encoder position to wheel angle units
  return (f32(ROTATION_DEG) / f32(ROTATION_MAX));
}

s16 DamperEffect (f32 spd, s16 mag) {
  //milos, speed in the units of wheel_angle/time_step
  if (spd > SPD_THRESHOLD)
    return (-ConstrainEffect(((spd - SPD_THRESHOLD) * mag * wDegScl()) * DAMPER_COEF * 10 / 512)); //milos
  if (spd < -SPD_THRESHOLD)
    return (-ConstrainEffect(((spd + SPD_THRESHOLD) * mag * wDegScl()) * DAMPER_COEF * 10 / 512)); //milos
  return (0);
}

s16 InertiaEffect(f32 acl, s16 mag) { //milos, modified
  //milos, acceleration in the units of wheel_angle/time_step^2
  //s16 cmd = ConstrainEffect(mag * abs(acl) * INERTIA_COEF / 32); //milos, my new
  s16 cmd = ConstrainEffect(mag * abs(acl) * wDegScl() * INERTIA_COEF * 10 / 32); //milos
  if (acl > ACL_THRESHOLD)
    return (-cmd);
  if (acl < -ACL_THRESHOLD)
    return (cmd);
  return (0);
}

s16 FrictionEffect (f32 spd, s16 mag) { //milos, modified
  //milos, simplified friction force model (constant above treshold, otherwise linear)
  //milos, speed in the units of wheel_angle/time_step
  s16 cmd = mag * FRICTION_COEF / 32;
  if (spd * wDegScl() * 10 > FRC_THRESHOLD)
    return (-ConstrainEffect(cmd));
  if (spd * wDegScl() * 10 < -FRC_THRESHOLD)
    return (ConstrainEffect(cmd));
  return (-ConstrainEffect(spd * cmd * wDegScl() * 10 / FRC_THRESHOLD));
}

s32 SpringEffect (s32 err, s16 mag) { //milos, modified - normalized to wheel angle
  //return (-ConstrainEffect((s32)((s32)mag * err * SPRING_COEF / 256)));
  return (-ConstrainEffect((s32)((s32)mag * err * wDegScl() * SPRING_COEF * 10 / 256))); //milos, added - with wheel angle as metric
}

s16 SineEffect (s16 mag, u16 period, u8 phase, u16 t) { //milos
  t %= period; //milos, reset timer after period reached
  return ((s16)(((f32)mag) * sin(TWO_PI * (1.0 / (((f32)period) / 1000.0) * (((f32)t) / 1000.0) + (((f32)phase) / 256.0))))); //milos, t increments in each cycle
}

s8 sgn(f32 value) { //milos added, sign function
  if (value >= 0.0) return (1);
  if (value < 0.0) return (-1);
}

s16 linFunction (f32 k, f32 x, s32 n) { //milos added, linear function y=kx+n
  return ((s16)(k * x) + n);
}

s16 SquareEffect (s16 mag, u16 period, u8 phase, u16 t) { //milos, added
  t %= period; //milos, reset timer after period reached
  return ((s16)(((f32)mag) * sgn(sin(TWO_PI * (1.0 / (((f32)period) / 1000.0) * (((f32)t) / 1000.0) + (((f32)phase) / 256.0)))))); //milos
}

s16 TriangleEffect (s16 mag, u16 period, u8 phase, u16 t) { //milos, added
  phase += 64; //milos, moved phase by PI/4, starts from 0 with rising edge
  t += (u16)(((f32)phase) / 256.0 * period); //milos, phase shifts time
  t %= period; //milos, reset timer after period reached
  if ((((f32)t) / 1000.0) >= 0.0 && (((f32)t) / 1000.0) < (((f32)period) / 1000.0) / 2.0) {
    return (linFunction(4.0 * ((f32)mag) / (((f32)period) / 1000.0), ((f32)t) / 1000.0, -mag));
  } else if ((((f32)t) / 1000.0) >= (((f32)period) / 1000.0) / 2.0 && (((f32)t) / 1000.0) < (((f32)period) / 1000.0)) {
    t -= period / 2;
    return (linFunction(-4.0 * ((f32)mag) / (((f32)period) / 1000.0), ((f32)t) / 1000.0, mag));
  }
}

s16 SawtoothUpEffect (s16 mag, u16 period, u8 phase, u16 t) { //milos, added
  phase += 128; //milos, moved phase by PI/2, starts from 0 with rising edge
  t += (u16)(((f32)phase) / 256.0 * period); //milos, phase shifts time
  t %= period; //milos, reset timer after period reached
  if (t >= 0 && t < period) {
    return (linFunction(2.0 * ((f32)mag) / (((f32)period) / 1000.0), ((f32)t) / 1000.0, -mag));
  }
}

s16 SawtoothDownEffect (s16 mag, u16 period, u8 phase, u16 t) { //milos, added
  phase += 128; //milos, moved phase by PI/2, starts from 0 with falling edge
  t += (u16)(((f32)phase) / 256.0 * period);
  t %= period; //milos, reset timer after period reached
  if (t >= 0 && t < period) {
    return (linFunction(2.0 * ((f32)(-mag)) / (((f32)period) / 1000.0), ((f32)t) / 1000.0, mag));
  }
}

s16 RampEffect (s8 rStart, s8 rEnd, u16 rPeriod, u16 t) { //milos, added
  f32 rSlope;
  t %= rPeriod; //milos, reset timer after period reached
  rSlope = ((f32)((s32)(rEnd - rStart) * 256)) / (((f32)rPeriod) / 1000.0); //milos, ramp slope (units magnitude/seconds)
  return (linFunction(rSlope, ((f32)t) / 1000.0, (s16)rStart * 256));
}

s16 ApplyEnvelope (s16 metric, u16 t, u8 atLvl, u8 fdLvl, u16 atTime, u16 fdTime, u16 eDuration, u16 eStartDelay) { //milos, added - envelope block effect, start delay and duration
  f32 kA, kF;
  s16 nA;
  if (metric >= 0) { // for positive magnitudes
    kA = (f32)(metric - (s16)(atLvl * 128)) / ((f32)atTime); // postitive attack slope
    nA = (s16)(atLvl * 128); // postitive attack offset
    kF = (f32)((s16)(fdLvl * 128) - metric) / ((f32)fdTime); // postitive fade slope
  } else { // for negative magnitudes
    kA = (f32)(metric + (s16)(atLvl * 128)) / ((f32)atTime); // negative attack slope
    nA = -(s16)(atLvl * 128); // negative attack offset
    kF = (f32)(-metric - (s16)(fdLvl * 128)) / ((f32)fdTime); // negative fade slope
  }

  if (t >= eStartDelay && t < eDuration + eStartDelay) { // play effect after start delay with a given effect duration
    if (t >= eStartDelay && t < atTime + eStartDelay) { // attack
      return (linFunction(kA, t - eStartDelay, nA));
    } else if (t >= atTime + eStartDelay && t <= eDuration + eStartDelay - fdTime) { // magnitude
      return (metric);
    } else if (t > eDuration + eStartDelay - fdTime && t < eDuration + eStartDelay) { // fade
      return (linFunction(kF, t - eDuration - eStartDelay + fdTime, metric));
    }
  } else {
    return (0);
  }
}

s32 ScaleMagnitude (s32 eMag, u16 eGain, float eDivider) { //milos, added
  return ((s32)eMag * (s32)eGain / 32767 / eDivider); // normalizes magnitude to effect gain and all PWM modes
}

//--------------------------------------------------------------------------------------------------------

void SetIndex () {
  gIndexFound = true;
  // 	myEnc.set(ROTATION_MID);
}

float EffectDivider() { //milos, added, calculates effects divider in order to scale magnitudes equaly for all PWM modes
  return (32767.0 / float(TOP)); //milos, was 32767.0
}

//--------------------------------------------------------------------------------------------------------
//s32 cFFB::CalcTorqueCommand (s32 pos) { // milos, commented - old 1 axis input
//s32 cFFB::CalcTorqueCommands (s32 pos, s32 pos) { // milos, pos: x-axis, pos2: y-axis
s32v cFFB::CalcTorqueCommands (s32v *pos) { // milos, pointer struct agument, returns struct, pos->x: x-axis, pos->y: y-axis
  //if ((!Btest(pidState.status, ACTUATORS_ENABLED)) || (Btest(pidState.status, DEVICE_PAUSED)))
  //return(0);

  s32v command; // milos, added 2-axis FFB data
  command.x = s32(0);
#ifdef USE_TWOFFBAXIS
  command.y = s32(0);
#endif // end of 2 ffb axis
  if (pos != NULL) { // milos, this check is always required for pointers

    f32 spd = mSpeed.Update(pos->x);
    f32 acl = mAccel.Update(spd); //milos, added - acceleration

    if (gFFB.mAutoCenter) { // milos, desktop autocenter spring effect if no FFB from any app or game
      if (bitRead(effstate, 0)) {
        /*if (abs(pos->x) > 1)*/ command.x += SpringEffect(pos->x, AUTO_CENTER_SPRING / EffectDivider() * configCenterGain / 100); //milos, autocenter spring force is equal (scaled accordingly) for all PWM modes
      }
#ifdef USE_TWOFFBAXIS
      if (bitRead(effstate, 0)) {
        /*if (abs(pos->y) > 1)*/ command.y += SpringEffect(pos->y, AUTO_CENTER_SPRING / EffectDivider() * configCenterGain / 100); //milos, autocenter spring for yFFB axis
      }
#endif
    } else for (u8 id = FIRST_EID; id <= MAX_EFFECTS; id++) { // milos, if an app or game is sending FFB

        /*milos
          u8 state;  // see constants MEffectState_*
          u8 type;  // see constants USB_EFFECT_*
          u8 parameterBlockOffset; // milos, added
          u8 attackLevel, fadeLevel, deadBand, deadBand2, enableAxis; //milos, added deadBand, deadBand2 and enableAxis
          s8 rampStart, rampEnd; //milos, added
          u16 gain, period, direction; // samplingPeriod; // ms //milos, changed gain from u8 to u16, added samplingPeriod
          u16 duration, fadeTime, attackTime, startDelay; //milos, added attackTime and startDelay
          s16 magnitude, magnitude2, positiveSaturation;  //milos, added positiveSaturation and magnitude2
          s16 offset, offset2;   //milos, added offset2
          u8 phase; //milos, changed back to u8 from u16

          ef.magnitude has range -32767..32767 16bit logical (the same physical)
          ef.period has range 0..65535 16bit logical, 0-65535 physical 16bit, exp -3, unit s
          ef.phase has range 0..255 8bit logical, 0-359 physical 8bit, exp 0, unit deg
          ef.gain has range 0..32767 16bit logical (the same physical)
          ef.offset has range -32768..32767 16bit logical (the same physical)
          ef.duration has range 0..65535 16bit logical, 0-65535 physical 16bit, exp -3, unit s
          ef.startDelay has range 0..65535 16bit logical, 0-65535 physical 16bit, exp -3, unit s
          ef.attackTime has range 0..32767 16bit logical, 0-32767 physical 16bit, exp -3, unit s
          ef.fadeTime has range 0..32767 16bit logical, 0-32767 physical 16bit, exp -3, unit s
          ef.attackLevel has range 0..255 8bit logical, 0-32767 physical
          ef.fadeLevel has range 0..255 8bit logical, 0-32767 physical
          ef.rampStart has range -127..127 8bit logical, -32767 to 32767 physical
          ef.rampEnd has range -127..127 8bit logical, -32767 to 32767 physical
          ef.deadBand has range 0..255 8bit logical, 0 to 32767 physical
          ef.direction has range 0..32767 16bit logical, 0 to 35999 physical, exp -2, unit deg
        */

        volatile TEffectState &ef = gEffectStates[id];
        if (Btest(ef.state, MEffectState_Allocated | MEffectState_Playing)) {

          s32 mag = ScaleMagnitude(ef.magnitude, ef.gain, EffectDivider()); // milos, effects are scaled equaly for all PWM modes
#ifdef USE_TWOFFBAXIS
          s32 mag2 = ScaleMagnitude(ef.magnitude2, ef.gain, EffectDivider()); // milos, magnitude for yFFB
#endif // end of 2 ffb axis

          if (ef.period <= (CONTROL_PERIOD / 1000) * 2) { //milos, make sure to cap the max frequency (or to limit min period)
            ef.period = (CONTROL_PERIOD / 1000) * 2; //milos, do now allow periods less than 4ms (more than 250Hz wave we can not reproduce with 500Hz FFB calculation rate anyway)
          }

          switch (ef.type) {
            case USB_EFFECT_CONSTANT:
              command.x -= ConstrainEffect(ScaleMagnitude(ApplyEnvelope(ef.magnitude, effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay) //milos, added
                                           , ef.gain, EffectDivider())) * configConstantGain / 100; //milos, added
              if (bitRead(ef.enableAxis, 2)) { // milos, if direction is enabled (bit2 of enableAxis byte)
#ifdef USE_TWOFFBAXIS
                command.y += command.x * cos(TWO_PI * ef.direction / 32768.0); //milos, added - project force vector on yFFB-axis
#endif // end of 2 ffb axis
                command.x *= sin(TWO_PI * ef.direction / 32768.0); //milos, added - project force vector on xFFB-axis
              }
              //LogTextLf("_pro constant");
              break;
            case USB_EFFECT_RAMP:
              command.x -= ConstrainEffect(ScaleMagnitude(ApplyEnvelope(RampEffect(ef.rampStart, ef.rampEnd, ef.duration, effectTime[id - 1]), effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay) //milos, added
                                           , ef.gain, EffectDivider())); //milos, added
              //LogTextLf("_pro ramp");
              break;
            case USB_EFFECT_SINE:
              command.x += ScaleMagnitude((s32)ef.offset + SineEffect(ApplyEnvelope(ef.magnitude, effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay), ef.period, ef.phase, effectTime[id - 1]) //milos, added
                                          , ef.gain, EffectDivider()) * configPeriodicGain / 100; //milos, added
              if (bitRead(ef.enableAxis, 2)) { // milos, if direction is enabled (bit2 of enableAxis byte)
#ifdef USE_TWOFFBAXIS
                command.y += command.x * cos(TWO_PI * ef.direction / 32768.0); //milos, added - project force vector on yFFB-axis
#endif // end of 2 ffb axis
                command.x *= sin(TWO_PI * ef.direction / 32768.0); //milos, added - project force vector on xFFB-axis
              }
              //LogTextLf("_pro sine");
              break;
            case USB_EFFECT_SQUARE:
              command.x += ScaleMagnitude((s32)ef.offset + SquareEffect(ApplyEnvelope(ef.magnitude, effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay), ef.period, ef.phase, effectTime[id - 1]) //milos, added
                                          , ef.gain, EffectDivider()) * configPeriodicGain / 100; //milos, added
              //LogTextLf("_pro square");
              break;
            case USB_EFFECT_TRIANGLE:
              command.x += ScaleMagnitude((s32)ef.offset + TriangleEffect(ApplyEnvelope(ef.magnitude, effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay), ef.period, ef.phase, effectTime[id - 1]) //milos, added
                                          , ef.gain, EffectDivider()) * configPeriodicGain / 100; //milos, added
              //LogTextLf("_pro triangle");
              break;
            case USB_EFFECT_SAWTOOTHUP:
              command.x += ScaleMagnitude((s32)ef.offset + SawtoothUpEffect(ApplyEnvelope(ef.magnitude, effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay), ef.period, ef.phase, effectTime[id - 1]) //milos, added
                                          , ef.gain, EffectDivider()) * configPeriodicGain / 100; //milos, added
              //LogTextLf("_pro sawtoothup");
              break;
            case USB_EFFECT_SAWTOOTHDOWN:
              command.x += ScaleMagnitude((s32)ef.offset + SawtoothDownEffect(ApplyEnvelope(ef.magnitude, effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay), ef.period, ef.phase, effectTime[id - 1]) //milos, added
                                          , ef.gain, EffectDivider()) * configPeriodicGain / 100; //milos, added
              //LogTextLf("_pro sawtoothdown");
              break;
            case USB_EFFECT_SPRING:
              command.x += SpringEffect(pos->x - (s16)((s32(ef.offset) * ROTATION_MID) >> 15), mag * configSpringGain / 100 / 16); //milos, for spring, damper, inertia and friction forces ef.offset is cpOffset, here we scale it to ROTATION_MID
#ifdef USE_TWOFFBAXIS //
              command.y += SpringEffect(pos->y - (s16)((s32(ef.offset2) * ROTATION_MID) >> 15), mag2 * configSpringGain / 100 / 16); //milos, for yFFB spring
#endif // end of 2 ffb axis
              //milos, with implemented cpOffset and dead band
              /*s16 mult;
                mult = 8;
                if (abs(pos->x - (s16)((s32(ef.offset) * ROTATION_MID) >> 15)) > (s16)ef.deadBand * mult) {
                if (pos->x - (s16)((s32(ef.offset) * ROTATION_MID) >> 15) >= 0) {
                  mult = -8;
                }
                command.x += SpringEffect(pos->x + (s16)ef.deadBand * mult - (s16)(((s32)ef.offset * ROTATION_MID) >> 15), mag * configSpringGain / 100 / 16); // milos, if 1 FFB axis apply condition0 to xFFB=f(pos->x)
                #ifdef USE_TWOFFBAXIS // milos, if 2 FFB axis apply condition1 to yFFB=f(pos->y)
                command.y += SpringEffect(pos->y + (s16)ef.deadBand2 * mult - (s16)(((s32)ef.offset2 * ROTATION_MID) >> 15), mag2 * configSpringGain / 100 / 16); //milos, spring on yFFB
                #endif // end of 2 ffb axis
                }*/
              //LogTextLf("_pro spring");
              break;
            case USB_EFFECT_DAMPER:
              command.x += DamperEffect(spd - (f32)ef.offset / 1638.3, mag * configDamperGain / 100) ; //milos, here we scale it to speed
              //milos, with implemented cpOffset and dead band
              /*if (abs(spd - (f32)ef.offset / 1638.3) > (f32)ef.deadBand / 32.0) {
                if (spd - (f32)ef.offset / 1638.3 >= 0) {
                  command.x += DamperEffect(spd - (f32)ef.deadBand / 32.0 - f32(ef.offset) / 1638.3, mag * configDamperGain / 100); //milos
                } else {
                  command.x += DamperEffect(spd + (f32)ef.deadBand / 32.0 - f32(ef.offset) / 1638.3, mag * configDamperGain / 100); //milos
                }
                } else {
                command.x += 0;
                }*/
              //LogTextLf("_pro damper");
              break;
            case USB_EFFECT_INERTIA:
              command.x += InertiaEffect(acl - (f32)ef.offset / 32767.0, mag * configInertiaGain / 100); //milos, here we scale it to acceleration
              //milos, with implemented cpOffset and dead band
              /*if (abs(acl - (f32)ef.offset / 32767.0) > (f32)ef.deadBand / 640.0) {
                if (acl - (f32)ef.offset / 32767.0 >= 0) {
                  command.x += InertiaEffect(acl - (f32)ef.deadBand / 640.0 - f32(ef.offset) / 32767.0, mag * configInertiaGain / 100); //milos
                } else {
                  command.x += InertiaEffect(acl + (f32)ef.deadBand / 640.0 - f32(ef.offset) / 32767.0, mag * configInertiaGain / 100); //milos
                }
                } else {
                command.x += 0;
                }*/
              //LogTextLf("_pro inertia");
              break;
            case USB_EFFECT_FRICTION:
              command.x += FrictionEffect(spd - (f32)ef.offset / 1638.3, mag * configFrictionGain / 100);
              //milos, with implemented dead band
              /*if (abs(spd - (f32)ef.offset / 1638.3) > (f32)ef.deadBand / 32.0) {
                if (spd - (f32)ef.offset / 1638.3 >= 0) {
                  command.x += FrictionEffect(spd - (f32)ef.deadBand / 32.0 - f32(ef.offset) / 1638.3, mag * configFrictionGain / 100); //milos
                } else {
                  command.x += FrictionEffect(spd + (f32)ef.deadBand / 32.0 - f32(ef.offset) / 1638.3, mag * configFrictionGain / 100); //milos
                }
                } else {
                command.x += 0;
                }*/
              //LogTextLf("_pro friction");
              break;
            //case USB_EFFECT_CUSTOM: //milos, commented
            //break;
            case USB_EFFECT_PERIODIC:
              command.x -= ConstrainEffect(ScaleMagnitude(ef.offset, 32767, EffectDivider())) * configPeriodicGain / 100; //milos, for periodic forces ef.offset changes magnitude, here we scale it to all PWM modes
              //LogTextLf("_pro periodic");
              break;
            default:
              break;
          }
          effectTime[id - 1] = millis() - t0; //milos, added - advance FFB timer
        }
      }
    // milos, at the moment only xFFB axis has conditional desktop (internal) effects
    if (bitRead(effstate, 1)) command.x += DamperEffect(spd, ScaleMagnitude(327 * configDamperGain, 32767, EffectDivider())) ; //milos, added - user damper effect
    if (bitRead(effstate, 2)) command.x += InertiaEffect(acl, ScaleMagnitude(327 * configInertiaGain, 32767, EffectDivider())) ; //milos, added - user inertia effect
    if (bitRead(effstate, 3)) command.x += FrictionEffect(spd, ScaleMagnitude(327 * configFrictionGain, 32767, EffectDivider())) ; //milos, added - user friction effect

    s32 limit = ROTATION_MID; // milos, +-ROTATION_MID distance from center is where endstop spring force will start
    //if ((pos->x < -limit) || (pos->x > limit)) {
    limit -= (ROTATION_MID >> 6); //milos, here you can offset endstop limit by ROTATION_MIN/64 (encoder can go past the limit)
    if ((pos->x < -limit) || (pos->x > limit)) {
      if (pos->x >= 0) {
        pos->x = pos->x - limit; //milos
      } else {
        pos->x = pos->x + limit; //milos
      }
      command.x += SpringEffect(pos->x, BOUNDARY_SPRING / EffectDivider() * configStopGain / 100); //milos, boundary spring force is equal (scaled accordingly) for all PWM modes, endstop force for xFFB axis
    }
    //}
#ifdef USE_TWOFFBAXIS // milos, add y-axis endstop with yFFB (at the moment y-axis is on the pot only)
    // milos, limit is the same as for xFFB endstop (analog axis are prescaled)
    //if ((pos->y < -limit) || (pos->y > limit)) {
    //limit -= 1023; // milos, maybe offset endstop limit a little (analog axis can't go past the limit, if left at 0 this force may never be created)
    if ((pos->y < -limit) || (pos->y > limit)) {
      if (pos->y >= 0) {
        pos->y = pos->y - limit; //milos
      } else {
        pos->y = pos->y + limit; //milos
      }
      command.y += SpringEffect(pos->y, BOUNDARY_SPRING / EffectDivider() * configStopGain / 100); //milos, boundary spring force is equal (scaled accordingly) for all PWM modes endstop force for yFFB axis
    }
    // }
#endif // end of 2 ffb axis

    command.x = ConstrainEffect(command.x * configGeneralGain / 100);
#ifndef USE_TWOFFBAXIS // milos, for 1 FFB axis
    if (bitRead(effstate, 4)) CONFIG_SERIAL.println(command.x); // milos, added - FFB real time monitor
#else // for 2 ffb axis
    command.y = ConstrainEffect(command.y * configGeneralGain / 100);
    if (bitRead(effstate, 4)) { // milos, for 2 ffb axis we send X and Y forces to FFB monitor
      CONFIG_SERIAL.print(command.x); // milos, FFB X axis
      CONFIG_SERIAL.print(" ");
      CONFIG_SERIAL.println(command.y); // milos, FFB Y axis
    }
    //torqueY = command2; // milos, added (commented out)
#endif // end of 2 ffb axis
  }
  return (command); // milos, passing the struct
}

//--------------------------------------------------------------------------------------------------------
/* Turn Steering right only */
void BRFFB::calibrate() {
  cal_print("cal:");
  s32 rightGap;
#ifdef USE_QUADRATURE_ENCODER
  rightGap = myEnc.Read();
#else
#ifdef USE_AS5600
  rightGap = as5600.getCumulativePosition();
#endif // end of as5600
#endif // end of quad enc
  s32 startpos = rightGap;
  s32 actual = 0;
  s32v drive; // milos, added - for setting PWM
  drive.x = 0; // milos, added - init at zero
#ifdef USE_TWOFFBAXIS
  drive.y = 0; // milos, added - init at zero
#endif // end of 2 ffb axis
#ifndef USE_ZINDEX //milos, added
  /* Turn right to stop and set MAX position */
  drive.x = (MM_MAX_MOTOR_TORQUE - MM_MIN_MOTOR_TORQUE) / 4;
  SetPWM(&drive);
  for (uint8_t i = 0; i < 254; i++) {
#ifdef USE_QUADRATURE_ENCODER
    actual = myEnc.Read();
#else
#ifdef USE_AS5600
    actual = as5600.getCumulativePosition();
#endif // end of as5600
#endif // end of quad enc
    if (rightGap >= actual) {
      break; // milos, if endstop reached
    } else {
      rightGap = actual;
    }
    delay(300);
  }
  //ROTATION_MAX = rightGap; // milos, commented
  drive.x = 0;
  SetPWM(&drive);
#ifndef USE_AS5600 // milos, when no AS5600
#ifdef USE_QUADRATURE_ENCODER
  myEnc.Write(ROTATION_MAX); // milos, set quadrature encoder to right edge
#endif // end of quad enc
#else
  as5600.resetCumulativePosition(ROTATION_MAX); // milos, set magnetic encoder to right edge
#endif
  if (startpos == actual) {
    cal_println("er");
    this->state = 2;
  } else {
    this->state = 1;
    cal_println("ok");  //milos
  }
#else // milos, if z-index is used
  // milos, added Z-index lookup
  // turn right at least 1 full turn or until we encounter Z-index pulse
#ifdef USE_QUADRATURE_ENCODER
  startpos = myEnc.Read();
  drive.x = (MM_MAX_MOTOR_TORQUE - MM_MIN_MOTOR_TORQUE) / 4;
  SetPWM(&drive);
#endif
  for (uint8_t i = 0; i < 254; i++) {
#ifdef USE_QUADRATURE_ENCODER
    actual = myEnc.Read();
#endif
    if (zIndexFound) {
      drive.x = 0;
      SetPWM(&drive);
      this->state = 1;
      cal_println("ok");
      break;
    }
    if ((actual - startpos) * ROTATION_DEG / ROTATION_MAX >= 360) {
      drive.x = 0;
      SetPWM(&drive);
      this->state = 3;
      cal_println("no z");
      break;
    }
    delay(50);
  }
#endif
  CONFIG_SERIAL.println(1); // milos, calibration procedure is done
}

BRFFB::BRFFB() {
  offset = 0;
  state = 0;
}

//--------------------------------------------------------------------------------------------------------

void FfbproSetAutoCenter(uint8_t enable)
{
  gFFB.mAutoCenter = enable;
  //brWheelFFB.autoCenter = true;
}

void FfbproEnableInterrupts(void)
{
}

const uint8_t* FfbproGetSysExHeader(uint8_t* hdr_len)
{
  static const uint8_t header[] = {0xf0, 0x00, 0x01, 0x0a, 0x01};
  *hdr_len = sizeof(header);
  return header;
}

// effect operations ---------------------------------------------------------

static void FfbproSendEffectOper(uint8_t effectId, uint8_t operation)
{
  /*
    uint8_t reportId; // =10
    uint8_t effectBlockIndex; // 1..40
    uint8_t operation; // 1=Start, 2=StartSolo, 3=Stop
    uint8_t loopCount; //0..255 (physical 0..255)
  */
}

void FfbproStartEffect(uint8_t effectId)
{
  //brWheelFFB.autoCenter = false;
  gFFB.mAutoCenter = false;
  effectTime[effectId - 1] = 0; //milos, added - reset timer for this effect when we start it
}

void FfbproStopEffect(uint8_t effectId)
{
  //setFFB(0); //milos, commented
  //brWheelFFB.autoCenter = false;
}

void FfbproFreeEffect(uint8_t effectId)
{
  //setFFB(0); //milos, commented
  //brWheelFFB.autoCenter = true;
}

// modify operations ---------------------------------------------------------

void FfbproModifyDuration(uint8_t effectId, uint16_t duration, uint16_t stdelay) //milos, added stdelay
{
  /*
    { // FFB: Set Effect Output Report
    uint8_t  reportId; // =1
    uint8_t effectBlockIndex; // 1..40
    uint8_t effectType; // 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28) //milos, total 11, 28 is removed (custom force)
    uint16_t duration; // 0..65535, exp -3, s
    uint16_t triggerRepeatInterval; // 0..65535, exp -3, s //milos
    int16_t gain; // 0..32767  (physical 0..32767) //milos, was 0(0)..(255)10000, uint8_t
    uint8_t triggerButton;  // button ID (0..8)
    uint8_t enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
    uint8_t direction; // angle (0=0 .. 255=359, exp 0, deg) //milos, 8bit
    uint16_t startDelay;  // 0..65535, exp -3, s //milos, uncommented
    } USB_FFBReport_SetEffect_Output_Data_t;
  */
  volatile TEffectState* effect = &gEffectStates[effectId]; // milos, added
  effect->duration = duration; // milos, added
  effect->startDelay = stdelay; // milos, added
  effectTime[effectId - 1] = 0; // milos, added - reset timer for this effect when duration is changed
}

/*void FfbproSetDeviceGain(USB_FFBReport_DeviceGain_Output_Data_t* data, volatile TEffectState * effect) //milos, added
  {

    //uint8_t  reportId; // =13
    //uint8_t deviceGain; //0..255  (physical 0..10000) //milos, back to 8bit

  effect->deviceGain = data->deviceGain; //milos, added
  }*/

void FfbproSetEnvelope (USB_FFBReport_SetEnvelope_Output_Data_t* data, volatile TEffectState * effect)
{
  uint8_t eid = data->effectBlockIndex;

  /*
    USB effect data:
    uint8_t  reportId; // =2
    uint8_t effectBlockIndex; // 1..40
    uint8_t attackLevel; // 0..255  (physical 0..32767) //milos, was 10000
    uint8_t fadeLevel; // 0..255  (physical 0..32767) //milos, was 10000
    uint16_t attackTime;  // 0..65535  (physical 0..65535), exp -3, s
    uint16_t fadeTime;  // 0..65535  (physical 0..65535), exp -3, s
  */
  effect->attackLevel = (u8)data->attackLevel;
  effect->fadeLevel = (u8)data->fadeLevel;
  effect->attackTime = (u16)data->attackTime; // milos, added
  effect->fadeTime = (u16)data->fadeTime;
}

void FfbproSetCondition (USB_FFBReport_SetCondition_Output_Data_t* data, volatile TEffectState * effect)
{
  uint8_t eid = data->effectBlockIndex;
  /*
    USB effect data:
  	uint8_t reportId; // =3
    uint8_t effectBlockIndex; // 1..40
    uint8_t parameterBlockOffset; // bits: 0..3=parameterBlockOffset, 4..5=instance1, 6..7=instance2
    int16_t cpOffset;  // -32768..32767 (physical -32768..32767) //milos, was -128(-10000)..127(10000), int8_t
    int16_t positiveCoefficient;  // -32767..32767 (physical -32767..32767) //milos, was -128(-10000)..127(10000), int8_t
    uint16_t positiveSaturation;  // 0..32767 (physical 0..32767) //milos, was 0(0)..255(10000), int8_t, uncommented
    uint8_t deadBand;  // 0..255 (physical 0..32767) //milos, was 0(0)..255(10000)
  */
  effect->parameterBlockOffset = (u8)data->parameterBlockOffset; // milos, added - pass the effect condition block index
  if (effect->parameterBlockOffset == 0) { // milos, condition block for xFFB
    effect->magnitude = (s16)data->positiveCoefficient; // milos, postitve coefficient can also be negative
    effect->offset = (s16)data->cpOffset; // milos, this offset changes X-pos
    effect->deadBand = (u8)data->deadBand; // milos, added
  } else if (effect->parameterBlockOffset == 1) { // milos, condition block for yFFB
#ifdef USE_TWOFFBAXIS
    effect->magnitude2 = (s16)data->positiveCoefficient; // milos, added  - yFFB spring constant
    effect->offset2 = (s16)data->cpOffset; // milos, this offset changes yFFB
    effect->deadBand2 = (u8)data->deadBand; // milos, added for yFFB
#endif // end of 2 ffb axis
  }
  //effect->positiveSaturation = (s16)data->positiveSaturation; // milos, posititve saturation can also be negative (not used currently)
}

void FfbproSetPeriodic (USB_FFBReport_SetPeriodic_Output_Data_t* data, volatile TEffectState * effect)
{
  uint8_t eid = data->effectBlockIndex;

  /*
  	typedef struct
  	{ // FFB: Set Periodic Output Report
    uint16_t magnitude; // 0..32767  (physical 0..32767) //milos, was 0(0)..255(10000), uint8_t
    int16_t offset; // -32768..32767  (physical -32768..32767) //milos, was -128(-10000)..(127)10000, int8_t
    uint8_t phase;  // 0..255 (physical 0..360, exp 0, deg) //milos, changed back to original uint8_t
    uint16_t period;  // 0..65535  (physical 0..65535), exp -3, s //milos, was 0(0)..32767(32767)
  	} USB_FFBReport_SetPeriodic_Output_Data_t;
  */
  //effect->type = USB_EFFECT_PERIODIC; // milos, was conflicting with FfbproSetEffect
  effect->magnitude = (u16)data->magnitude;
  effect->offset = (((s16)data->offset)); // milos, this offset changes magnitude
  effect->phase = (u8)data->phase;
  effect->period = (u16)data->period;
}

void FfbproSetConstantForce (USB_FFBReport_SetConstantForce_Output_Data_t* data, volatile TEffectState * effect)
{
  uint8_t eid = data->effectBlockIndex;
  /*
    USB data:
    uint8_t  reportId; // =5
    uint8_t effectBlockIndex; // 1..40
    int16_t magnitude;  // -32767..32737  (physical -32767..32737) //milos, logical was -255..255
  */
  effect->magnitude = data->magnitude;
}

void FfbproSetRampForce (USB_FFBReport_SetRampForce_Output_Data_t* data, volatile TEffectState * effect)
{
  uint8_t eid = data->effectBlockIndex; // milos, uncommented
  /*USB effect data:
    uint8_t	reportId;	// =6
    uint8_t	effectBlockIndex;	// 1..40
    int8_t rampStart; // -128..127  (physical -32768..32767) //milos, was -10000..10000
    int8_t rampEnd; // -128..127  (physical -32768..32767) //milos, was -10000..10000*/
  effect->rampStart = data->rampStart; // milos, added
  effect->rampEnd = data->rampEnd; // milos, added
}

void setFFB(s32 command) {
  //DEBUG_SERIAL.println("setffb");
  //DEBUG_SERIAL.println(command);
  /*if (command > 0)
  	command = map(command, 0, 1000, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE - 10);
    else if (command < 0)
  	command = -map(-command, 0, 1000, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE - 10);
    else
  	command = 0;*/
  //setPWMDir(command);
}

uint8_t FfbproSetEffect (USB_FFBReport_SetEffect_Output_Data_t* data, volatile TEffectState * effect)  //milos, changed from int to uint8_t
{
  /*
    {
    u8 state;  // see constants MEffectState_*
    u8 type;  // see constants USB_EFFECT_*
    u8 attackLevel, fadeLevel, deadBand, direction; //milos, added deadBand and direction
    s8 rampStart, rampEnd; //milos, added
    u16 gain, period; // samplingPeriod;  // ms //milos, changed gain from u8 to u16, added samplingPeriod
    u16 duration, fadeTime, attackTime, startDelay; //milos, added attackTime and startDelay
    s16 magnitude;
    s16 offset;
    u8 phase; //milos, changed back to u8 from u16
  */

  uint8_t eid = data->effectBlockIndex;
  //s32 command = s32(0);
  /*
    { // FFB: Set Effect Output Report
    uint8_t  reportId; // =1
    uint8_t effectBlockIndex; // 1..40
    uint8_t effectType; // 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28) //milos, total 11, 28 is removed (custom force)
    uint16_t duration; // 0..65535, exp -3, s
    uint16_t triggerRepeatInterval; // 0..65535, exp -3, s //milos
    int16_t gain; // 0..32767  (physical 0..32767) //milos, was 0(0)..(255)10000, uint8_t
    uint8_t triggerButton;  // button ID (0..8)
    uint8_t enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
    uint16_t direction; // angle (0=0 .. 65535=35999, exp -2, deg) //milos, 16bit
    uint16_t startDelay;  // 0..65535, exp -3, s //milos, uncommented
    } USB_FFBReport_SetEffect_Output_Data_t;
  */
  effect->type = data->effectType; // milos, this is where effect type is being set
  effect->gain = (s16)data->gain;
  FfbproModifyDuration(eid, (u16)data->duration, (u16)data->startDelay); // milos, added
  //effect->startDelay = (u16)data->startDelay; / /milos, added
  effect->direction = (u16)data->direction; // milos, added
  effect->enableAxis = (u8)data->enableAxis; // milos, added
  //bool is_periodic = false; //milos, commented
  //s32 mag = (((s32)effect->magnitude)*((s32)effect->gain)) / 163;
  // Fill in the effect type specific data
  /*switch (data->effectType)
    {
    case USB_EFFECT_SQUARE:
    case USB_EFFECT_SINE:
    case USB_EFFECT_TRIANGLE:
    case USB_EFFECT_SAWTOOTHDOWN:
    case USB_EFFECT_SAWTOOTHUP:
  	is_periodic = true;
    case USB_EFFECT_CONSTANT:
    case USB_EFFECT_RAMP:
    case USB_EFFECT_SPRING:
    case USB_EFFECT_DAMPER:
    case USB_EFFECT_INERTIA:
    case USB_EFFECT_FRICTION:
    case USB_EFFECT_CUSTOM:
    default:
  	break;
    }*/
  /*if (command > 0)
  	command = map(command, 0, 1000, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE - 10);
    else if (command < 0)
  	command = -map(-command, 0, 1000, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE - 10);
    else
  	command = 0;*/
  //setFFB(command);
  //DEBUG_SERIAL.println(command);
  return 1;
}

void FfbproCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, volatile TEffectState * effect)
{
  /*
    USB effect data:
    uint8_t		reportId;	// =1
    uint8_t	effectType;	// Enum (1..12): ET 26,27,30,31,32,33,34,40,41,42,43,28
    uint16_t	byteCount;	// 0..511	- only valid with Custom Force
  */
}
