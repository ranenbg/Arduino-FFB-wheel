/*
  Force Feedback Joystick
  USB HID descriptors for a force feedback joystick.

  This code is for Microsoft Sidewinder Force Feedback Pro joystick.
  with some room for additional extra controls.

  Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
  Copyright 2018-2025  Milos Rankovic (ranenbg [at] gmail [dot] com)
  MIT License.

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

#ifndef _FFB_
#define _FFB_

#include <Arduino.h>
#include "Config.h" // milos, added

/* Type Defines: */
/** Type define for the joystick HID report structure, for creating and sending HID reports to the host PC.
    This mirrors the layout described to the host in the HID report descriptor, in Descriptors.c.
*/

// Maximum number of parallel effects in memory
#define MAX_EFFECTS 11 //milos, changed from 20

// milos, this will increment in each cycle by 2ms, 500Hz FFB effects calculation
// bare in mind the Nyquist sampling frequency, for 500Hz we can reproduce up to 250Hz wave (4ms period)
// that should be more than enough for all vibrational effects
u32 t0 = 0; //milos, added
u32 effectTime[MAX_EFFECTS]; //milos, added - ffb calculation timer (effect elapsed playing time in ms, max is 65535)
bool t0_updated = false; //milos, added - keeps track if we updated zero time when effects start

// ---- Input

typedef struct
{
  uint8_t	reportId;	// =2
  uint8_t	status;	// Bits: 0=Device Paused,1=Actuators Enabled,2=Safety Switch,3=Actuator Override Switch,4=Actuator Power
  uint8_t	effectBlockIndex;	// Bit7=Effect Playing, Bit0..7=EffectId (1..40)
} USB_FFBReport_PIDStatus_Input_Data_t;

// ---- Output

typedef struct
{ // FFB: Set Effect Output Report
  uint8_t	reportId;	// =1
  uint8_t	effectBlockIndex;	// 1..40
  uint8_t	effectType;	// 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28) //milos, total 11, 28 is removed (custom force)
  uint16_t duration; // 0..65535, exp -3, s
  uint16_t triggerRepeatInterval; // 0..65535, exp -3, s // milos, do not comment out this (ffb stops working)
  //uint16_t samplingPeriod;	// 0..65535, exp -3, s //milos, renamed from samplePeriod, removed
  int16_t gain; // 0..32767  (physical 0..32767) //milos, was 0(0)..(255)10000, uint8_t
  uint8_t	triggerButton;	// button ID (0..8) // milos, do not comment out this (ffb stops working)
  uint8_t	enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
  //uint8_t	directionX;	// angle (0=0 .. 255=360deg) //milos, commented
  //uint8_t	directionY;	// angle (0=0 .. 255=360deg) //milos, commented
  //uint8_t direction; // angle (0=0 .. 255=359, exp 0, deg) //milos, 8bit
  uint16_t direction; // angle (0=0 .. 32767=35999, exp -2, deg) //milos, 16bit
  uint16_t startDelay;	// 0..65535, exp -3, s //milos, uncommented
} USB_FFBReport_SetEffect_Output_Data_t;

typedef struct
{ // FFB: Set Envelope Output Report
  uint8_t	reportId;	// =2
  uint8_t	effectBlockIndex;	// 1..40
  uint8_t attackLevel; // 0..255  (physical 0..32767) //milos, was 10000
  uint8_t	fadeLevel; // 0..255  (physical 0..32767) //milos, was 10000
  uint16_t attackTime;	// 0..32767  (physical 0..32767), exp -3, s
  uint16_t fadeTime;	// 0..32767  (physical 0..32767), exp -3, s
} USB_FFBReport_SetEnvelope_Output_Data_t;

typedef struct
{ // FFB: Set Condition Output Report
  uint8_t	reportId;	// =3
  uint8_t	effectBlockIndex;	// 1..40
  uint8_t	parameterBlockOffset;	// bits: 0..3=parameterBlockOffset, 4..5=instance1, 6..7=instance2
  int16_t cpOffset;	// -32768..32767 (physical -32768..32767) //milos, was -128(-10000)..127(10000), int8_t
  int16_t	positiveCoefficient;	// -32767..32767 (physical -32767..32767) //milos, was -128(-10000)..127(10000), int8_t
  //  int16_t	negativeCoefficient;	// -32768..32767 (physical -32768..32767) //milos, was -128(-10000)..127(10000), int8_t, commented out
  //  int16_t positiveSaturation;	// 0..32767 (physical 0..32767) //milos, was 0(0)..255(10000), int8_t, commented out
  //  uint8_t	negativeSaturation;	// -128..127
  uint8_t deadBand;	// 0..255 (physical 0..32767) //milos, was 0(0)..255(10000)
} USB_FFBReport_SetCondition_Output_Data_t;

typedef struct
{ // FFB: Set Periodic Output Report
  uint8_t	reportId;	// =4
  uint8_t	effectBlockIndex;	// 1..40
  int16_t magnitude; // 0..32767  (physical 0..32767) //milos, was 0(0)..255(10000), uint8_t
  int16_t offset; // -32768..32767  (physical -32768..32767) //milos, was -128(-10000)..(127)10000, int8_t
  //uint16_t phase;	// 0..65535 (physical 0..36000, exp-2, deg) //milos, was 0(0)..(255)36000, uint8_t
  uint8_t phase;  // 0..255 (physical 0..359, exp 0, deg) //milos, changed back to original uint8_t
  uint16_t period;	// 0..65535  (physical 0..65535), exp -3, s //milos, was 0(0)..32767(32767)
} USB_FFBReport_SetPeriodic_Output_Data_t;

typedef struct
{ // FFB: Set ConstantForce Output Report
  uint8_t	reportId;	// =5
  uint8_t	effectBlockIndex;	// 1..40
  int16_t magnitude;	// -32767..32737  (physical -32767..32737) //milos, logical was -255..255
} USB_FFBReport_SetConstantForce_Output_Data_t;

typedef struct
{ // FFB: Set RampForce Output Report
  uint8_t	reportId;	// =6
  uint8_t	effectBlockIndex;	// 1..40
  int8_t rampStart; // -127..127  (physical -32767..32767) //milos, was -10000..10000
  int8_t rampEnd; // -127..127  (physical -32767..32767) //milos, was -10000..10000
} USB_FFBReport_SetRampForce_Output_Data_t;

//milos, commented since it was not used
/*typedef struct
  { // FFB: Set CustomForceData Output Report
  uint8_t	reportId;	// =7
  uint8_t	effectBlockIndex;	// 1..40
  //uint8_t dataOffset; //milos, commented
  int16_t dataOffset; // -32768..32767  (physical -32768..32767) //milos, was 0(0)..255(10000), was 16bit in HID.cpp
  int8_t data[12]; // -127..127  (physical -32767..32767) //milos, physical was 0..255
  } USB_FFBReport_SetCustomForceData_Output_Data_t;*/

//milos, commented since it was not used
/*typedef struct
  { // FFB: Set DownloadForceSample Output Report
  uint8_t	reportId;	// =8
  int8_t	x; // -127..127  (physical -32767..32767) //milos, physical was 0..255
  int8_t	y; // -127..127  (physical -32767..32767) //milos, physical was 0..255
  } USB_FFBReport_SetDownloadForceSample_Output_Data_t;*/

typedef struct
{ // FFB: Set EffectOperation Output Report
  uint8_t	reportId;	// =10
  uint8_t effectBlockIndex;	// 1..40
  uint8_t operation; // 1=Start, 2=StartSolo, 3=Stop
  uint8_t	loopCount; //0..255 (physical 0..255)
} USB_FFBReport_EffectOperation_Output_Data_t;

typedef struct
{ // FFB: Block Free Output Report
  uint8_t	reportId;	// =11
  uint8_t effectBlockIndex;	// 1..40
} USB_FFBReport_BlockFree_Output_Data_t;

typedef struct
{ // FFB: Device Control Output Report
  uint8_t	reportId;	// =12
  uint8_t control;	// 1=Enable Actuators, 2=Disable Actuators, 4=Stop All Effects, 8=Reset, 16=Pause, 32=Continue
} USB_FFBReport_DeviceControl_Output_Data_t;

typedef struct
{ // FFB: DeviceGain Output Report
  uint8_t	reportId;	// =13
  //uint16_t deviceGain; //0..32767  (physical 0..32767) //milos, was 0(0)..255(10000), uint8_t
  uint8_t deviceGain; //0..255  (physical 0..10000) //milos, back to 8bit
} USB_FFBReport_DeviceGain_Output_Data_t;

//milos, commented since it was not used
/*typedef struct
  { // FFB: Set Custom Force Output Report
  uint8_t	reportId;	// =14
  uint8_t effectBlockIndex;	// 1..40
  uint8_t	sampleCount; // 0..255
  uint16_t samplePeriod;	// 0..32767 ms
  } USB_FFBReport_SetCustomForce_Output_Data_t;*/

// ---- Features

typedef struct
{ // FFB: Create New Effect Feature Report
  uint8_t reportId;	// =1
  uint8_t	effectType;	// Enum (1..12): ET 26,27,30,31,32,33,34,40,41,42,43 //,28 //milos, total 11, 28 custom force is removed
  uint16_t byteCount;	// 0..511
} USB_FFBReport_CreateNewEffect_Feature_Data_t;

typedef struct
{ // FFB: PID Block Load Feature Report
  uint8_t	reportId;	// =2
  uint8_t effectBlockIndex;	// 1..40
  uint8_t	loadStatus;	// 1=Success,2=Full,3=Error
  uint16_t ramPoolAvailable;	// =0 or 0xFFFF?
} USB_FFBReport_PIDBlockLoad_Feature_Data_t;

typedef struct
{ // FFB: PID Pool Feature Report
  uint8_t	reportId;	// =3
  uint16_t ramPoolSize;	// 65535
  uint8_t	maxSimultaneousEffects;	// ?? 40?
  uint8_t	memoryManagement;	// Bits: 0=DeviceManagedPool, 1=SharedParameterBlocks
} USB_FFBReport_PIDPool_Feature_Data_t;

// Lengths of each report type
extern const uint16_t OutReportSize[];

// Handles Force Feeback data manipulation from USB reports to joystick's MIDI channel

void FfbSetDriver(uint8_t id);

// Initializes and enables MIDI to joystick using USART1 TX
void FfbInitMidi(void);

// Send "enable FFB" to joystick
void FfbSendEnable(void);

// Send "disable FFB" to joystick
void FfbSendDisable(void);

// Handle incoming data from USB
void FfbOnUsbData(uint8_t *data, uint16_t len);

// Handle incoming feature requests
void FfbOnCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, USB_FFBReport_PIDBlockLoad_Feature_Data_t *outData);
void FfbOnPIDPool(USB_FFBReport_PIDPool_Feature_Data_t *data);

// Utility to wait any amount of milliseconds.
// Resets watchdog for each 1ms wait.
void WaitMs(int ms);

// delay_us has max limits and the wait time must be known at compile time.
// function for making 10us delays that don't have be known at compile time.
// max delay 2560us.
void _delay_us10(uint8_t delay);

// Send raw data to the
void FfbSendData(const uint8_t *data, uint16_t len);
void FfbSendPackets(const uint8_t *data, uint16_t len);
void FfbPulseX1( void );

// Debugging
//	<index> should be pointer to an index variable whose value should be set to 0 to start iterating.
//	Returns 0 when no more effects
uint8_t FfbDebugListEffects(uint8_t *index);

// Effect manipulations

typedef struct
{
  uint8_t midi;	// disables all MIDI-traffic
  uint8_t springs;
  uint8_t constants;
  uint8_t triangles;
  uint8_t sines;
  uint8_t effectId[MAX_EFFECTS];
} TDisabledEffectTypes;

extern volatile TDisabledEffectTypes gDisabledEffects;

void FfbSendSysEx(const uint8_t* midi_data, uint8_t len);

void FfbEnableSprings(uint8_t inEnable);
void FfbEnableConstants(uint8_t inEnable);
void FfbEnableTriangles(uint8_t inEnable);
void FfbEnableSines(uint8_t inEnable);
void FfbEnableEffectId(uint8_t inId, uint8_t inEnable);

// Bit-masks for effect states
#define MEffectState_Free			0x00
#define MEffectState_Allocated		0x01
#define MEffectState_Playing		0x02

#define USB_DURATION_INFINITE	0xFFFF //milos, changed to 0xFFFF or 65535, was 0x7FFF or 32767

#define USB_EFFECT_CONSTANT		0x01
#define USB_EFFECT_RAMP			0x02
#define USB_EFFECT_SQUARE 		0x03
#define USB_EFFECT_SINE 		0x04
#define USB_EFFECT_TRIANGLE		0x05
#define USB_EFFECT_SAWTOOTHDOWN	0x06
#define USB_EFFECT_SAWTOOTHUP	0x07
#define USB_EFFECT_SPRING		0x08
#define USB_EFFECT_DAMPER		0x09
#define USB_EFFECT_INERTIA		0x0A
#define USB_EFFECT_FRICTION		0x0B
#define USB_EFFECT_CUSTOM		0x0C
#define USB_EFFECT_PERIODIC		0x0D

typedef struct
{
  u8 state;	// see constants MEffectState_*
  u8 type;	// see constants USB_EFFECT_*
  u8 parameterBlockOffset; // milos, added
  u8 attackLevel, fadeLevel, deadBand, enableAxis; //milos, added deadBand and enableAxis
  s8 rampStart, rampEnd; //milos, added
  u16 gain, period, direction; // samplingPeriod;	// ms //milos, changed gain from u8 to u16, added samplingPeriod
  u16 duration, fadeTime, attackTime, startDelay; //milos, added attackTime and startDelay
  s16 magnitude, positiveCoefficient;  //milos, added positiveCoefficient
  s16 offset;
  u8 phase; //milos, changed back to u8 from u16
#ifdef USE_TWOFFBAXIS // milos, added - used for conditional block effects for yFFB axis
  u8 deadBand2;
  s16 magnitude2, offset2;
#endif // end of 2 ffb axis
} TEffectState;

typedef struct
{
  void (*EnableInterrupts)(void);
  const uint8_t* (*GetSysExHeader)(uint8_t* hdr_len);
  void (*SetAutoCenter)(uint8_t enable);

  void (*StartEffect)(uint8_t eid);
  void (*StopEffect)(uint8_t eid);
  void (*FreeEffect)(uint8_t eid);

  void (*ModifyDuration)(uint8_t effectId, uint16_t duration);
  //void (*SetDeviceGain)(USB_FFBReport_DeviceGain_Output_Data_t* data, volatile TEffectState* effect); //milos, added

  void (*CreateNewEffect)(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, volatile TEffectState* effect);
  void (*SetEnvelope)(USB_FFBReport_SetEnvelope_Output_Data_t* data, volatile TEffectState* effect);
  void (*SetCondition)(USB_FFBReport_SetCondition_Output_Data_t* data, volatile TEffectState* effect);
  void (*SetPeriodic)(USB_FFBReport_SetPeriodic_Output_Data_t* data, volatile TEffectState* effect);
  void (*SetConstantForce)(USB_FFBReport_SetConstantForce_Output_Data_t* data, volatile TEffectState* effect);
  void (*SetRampForce)(USB_FFBReport_SetRampForce_Output_Data_t* data, volatile TEffectState* effect);
  uint8_t (*SetEffect)(USB_FFBReport_SetEffect_Output_Data_t* data, volatile TEffectState* effect);  //milos, changed from int to uint8_t
} FFB_Driver;

#endif // _FFB_PRO_
