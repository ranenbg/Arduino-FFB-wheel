/*
  Force Feedback Wheel

  Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
  Copyright 2013  Saku Kekkonen
  Copyright 2015  Etienne Saint-Paul  (esaintpaul [at] gameseed [dot] fr)
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
#include <arduino.h>
#include "USBCore.h"
#include <stdint.h>
#include "debug.h"
#include "ffb_pro.h"
#include "ConfigHID.h"

//------------------------------------- Defines ----------------------------------------------------------
u8 valueglobal = 55;
#define LEDs_SetAllLEDs(l)

#define FIRST_EID	1

//--------------------------------------- Globals --------------------------------------------------------

const FFB_Driver ffb_drivers[1] =
{
  {
    FfbproEnableInterrupts,
    FfbproGetSysExHeader,
    FfbproSetAutoCenter,
    FfbproStartEffect,
    FfbproStopEffect,
    FfbproFreeEffect,
    FfbproModifyDuration,
    FfbproCreateNewEffect,
    FfbproSetEnvelope,
    FfbproSetCondition,
    FfbproSetPeriodic,
    FfbproSetConstantForce,
    FfbproSetRampForce,
    FfbproSetEffect,
    //FfbproSetDeviceGain, //milos, added
  }
};

static const FFB_Driver* ffb;

void setFFB(s32 command);

// Effect management
volatile uint8_t nextEID = FIRST_EID;	// FFP effect indexes starts from 1
volatile USB_FFBReport_PIDStatus_Input_Data_t pidState;	// For holding device status flags

void SendPidStateForEffect(uint8_t eid, uint8_t effectState);
void SendPidStateForEffect(uint8_t eid, uint8_t effectState)
{
  pidState.effectBlockIndex = effectState;
  pidState.effectBlockIndex = 0;
}

static volatile TEffectState gEffectStates[MAX_EFFECTS + 1];	// one for each effect (array index 0 is unused to simplify things)

volatile TDisabledEffectTypes gDisabledEffects;
USB_FFBReport_PIDBlockLoad_Feature_Data_t gNewEffectBlockLoad;

uint8_t GetNextFreeEffect(void);
void StartEffect(uint8_t id);
void StopEffect(uint8_t id);
void StopAllEffects(void);
void FreeEffect(uint8_t id);
void FreeAllEffects(void);

//-------------------------------------------------------------------------------------------------------------

#if defined(__AVR_ATmega32U4__)										// On arduino uno we don't have USB

b8 HID_GetReport (Setup& setup)
{
  u8 report_id = setup.wValueL;
  u8 report_type = setup.wValueH;
  if ((report_id == 6))// && (gNewEffectBlockLoad.reportId==6))
  {
    USB_SendControl(TRANSFER_RELEASE, &gNewEffectBlockLoad, sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t));
    gNewEffectBlockLoad.reportId = 0;
    return (true);
  }
  if (report_id == 7)
  {
    USB_FFBReport_PIDPool_Feature_Data_t ans;
    ans.reportId = report_id;
    ans.ramPoolSize = 0xffff;
    ans.maxSimultaneousEffects = MAX_EFFECTS;
    ans.memoryManagement = 3;
    USB_SendControl(TRANSFER_RELEASE, &ans, sizeof(USB_FFBReport_PIDPool_Feature_Data_t));
    return (true);
  }
  return (false);
}

b8 HID_SetReport (Setup& setup)
{
  u8 report_id = setup.wValueL;
  u8 report_type = setup.wValueH;
  if (report_id == 5)
  {
    USB_FFBReport_CreateNewEffect_Feature_Data_t ans;
    USB_RecvControl(&ans, sizeof(USB_FFBReport_CreateNewEffect_Feature_Data_t));
    FfbOnCreateNewEffect(&ans, &gNewEffectBlockLoad);
  }

  return (true);
}

b8 FFB_HID_Setup (Setup& setup)
{
  u8 r = setup.bRequest;
  u8 requestType = setup.bmRequestType;
  if (REQUEST_DEVICETOHOST_CLASS_INTERFACE == requestType)
  {
    if (HID_GET_REPORT == r)
    {
      HID_GetReport(setup);
      return true;
    }
    if (HID_GET_PROTOCOL == r)
    {
      //Send8(_hid_protocol);	// TODO
      return true;
    }
  }

  if (REQUEST_HOSTTODEVICE_CLASS_INTERFACE == requestType)
  {
    switch (r)
    {
      case HID_SET_PROTOCOL:
        //			_hid_protocol = setup.wValueL;
        return true;

      case HID_SET_IDLE:
        //			_hid_idle = setup.wValueL;
        return true;

      case HID_SET_REPORT:
        HID_SetReport(setup);
        return true;
    }
  }
  return (true);
  //	return false;
}

//-------------------------------------------------------------------------------------------------------------

void FfbSetDriver(uint8_t id)
{
  ffb = &ffb_drivers[id];
  USBDevice.HID_Setup_Callback = FFB_HID_Setup;
  USBDevice.HID_ReceiveReport_Callback = FfbOnUsbData;
}

uint8_t GetNextFreeEffect(void)
{
  if (nextEID == MAX_EFFECTS)
    return 0;

  uint8_t id = nextEID++;

  // Find the next free effect ID for next time
  while (gEffectStates[nextEID].state != 0)
  {
    if (nextEID >= MAX_EFFECTS)
      break;	// the last spot was taken
    nextEID++;
  }
  gEffectStates[id].state = MEffectState_Allocated;
  return id;
}

void StopAllEffects(void)
{
  LogTextLf("FFB.ino StopAllEffects");
  for (uint8_t id = FIRST_EID; id <= MAX_EFFECTS; id++)
    StopEffect(id);
}

void StartEffect(uint8_t id)
{
  if ((id > MAX_EFFECTS) || (gEffectStates[id].state == 0))
    return;
  gEffectStates[id].state |= MEffectState_Playing;
  //milos, added - update zero time only when first effect starts
  if (!t0_updated) {
    t0 = millis();
    t0_updated = true;
  }
}

void StopEffect(uint8_t id)
{
  if ((id > MAX_EFFECTS) || (gEffectStates[id].state == 0))
    return;
  gEffectStates[id].state &= ~MEffectState_Playing;
  if (!gDisabledEffects.effectId[id]) {
    ffb->StopEffect(id);
    t0_updated = false; //milos, added
  }
}

void FreeEffect(uint8_t id)
{
  gFFB.mAutoCenter = true;
  if (id > MAX_EFFECTS)
    return;
  gEffectStates[id].state = 0;
  if (id < nextEID)
    nextEID = id;
  ffb->FreeEffect(id);
  t0_updated = false; //milos, added
}

void FreeAllEffects(void)
{
  nextEID = FIRST_EID;
  memset((void*) gEffectStates, 0, sizeof(gEffectStates));
  LogTextLf("FFB.ino FreeAllEffects");
}

// Lengths of each report type
const uint16_t OutReportSize[] =
{
  sizeof(USB_FFBReport_SetEffect_Output_Data_t),		// 1
  sizeof(USB_FFBReport_SetEnvelope_Output_Data_t),	// 2
  sizeof(USB_FFBReport_SetCondition_Output_Data_t),	// 3
  sizeof(USB_FFBReport_SetPeriodic_Output_Data_t),	// 4
  sizeof(USB_FFBReport_SetConstantForce_Output_Data_t),	// 5
  sizeof(USB_FFBReport_SetRampForce_Output_Data_t),	// 6
  0,//sizeof(USB_FFBReport_SetCustomForceData_Output_Data_t),	// 7 //milos, commented
  0,//sizeof(USB_FFBReport_SetDownloadForceSample_Output_Data_t),	// 8 //milos, commented
  0,	// 9
  sizeof(USB_FFBReport_EffectOperation_Output_Data_t),	// 10
  sizeof(USB_FFBReport_BlockFree_Output_Data_t),	// 11
  sizeof(USB_FFBReport_DeviceControl_Output_Data_t),	// 12
  sizeof(USB_FFBReport_DeviceGain_Output_Data_t),	// 13
  0,//sizeof(USB_FFBReport_SetCustomForce_Output_Data_t),	// 14 //milos, commented
};

void FfbHandle_EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t *data);
void FfbHandle_BlockFree(USB_FFBReport_BlockFree_Output_Data_t *data);
void FfbHandle_DeviceControl(USB_FFBReport_DeviceControl_Output_Data_t *data);
void FfbHandle_DeviceGain(USB_FFBReport_DeviceGain_Output_Data_t *data);
//void FfbHandle_SetCustomForceData(USB_FFBReport_SetCustomForceData_Output_Data_t* data); //milos, commented since it was not used
//void FfbHandle_SetDownloadForceSample(USB_FFBReport_SetDownloadForceSample_Output_Data_t* data); //milos, commented since it was not used
//void FfbHandle_SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t* data); //milos, commented since it was not used
void FfbHandle_SetEffect(USB_FFBReport_SetEffect_Output_Data_t *data);

// Handle incoming data from USB
void FfbOnUsbData(uint8_t *data, uint16_t len)
{
  // Parse incoming USB data
  LEDs_SetAllLEDs(LEDS_ALL_LEDS);

  uint8_t effectId = data[1]; // effectBlockIndex is always the second byte.

  switch (data[0])	// reportID
  {
    case 1:
      FfbHandle_SetEffect((USB_FFBReport_SetEffect_Output_Data_t *) data);
      //milos, added
      LogText("SetEff - index:");
      LogBinary(&((USB_FFBReport_SetEffect_Output_Data_t*)data)->effectBlockIndex, 1);
      LogText(", type:");
      LogBinary(&((USB_FFBReport_SetEffect_Output_Data_t*)data)->effectType, 1);
      LogText(", gain:");
      LogBinary(&((USB_FFBReport_SetEffect_Output_Data_t*)data)->gain, 2);
      LogText(", dur:");
      LogBinary(&((USB_FFBReport_SetEffect_Output_Data_t*)data)->duration, 2);
      LogText(", dly:");
      LogBinary(&((USB_FFBReport_SetEffect_Output_Data_t*)data)->startDelay, 2);
      LogText(", dir:");
      LogBinary(&((USB_FFBReport_SetEffect_Output_Data_t*)data)->direction, 2);
      LogText(", axis:");
      LogBinaryLf(&((USB_FFBReport_SetEffect_Output_Data_t*)data)->enableAxis, 1);
      break;
    case 2:
      ffb->SetEnvelope((USB_FFBReport_SetEnvelope_Output_Data_t*) data, &gEffectStates[effectId]);
      //milos, added
      LogText("SetEnv - aL:");
      LogBinary(&((USB_FFBReport_SetEnvelope_Output_Data_t*)data)->attackLevel, 1);
      LogText(", aT:");
      LogBinary(&((USB_FFBReport_SetEnvelope_Output_Data_t*)data)->attackTime, 2);
      LogText(", fL:");
      LogBinary(&((USB_FFBReport_SetEnvelope_Output_Data_t*)data)->fadeLevel, 1);
      LogText(", fT:");
      LogBinaryLf(&((USB_FFBReport_SetEnvelope_Output_Data_t*)data)->fadeTime, 2);
      break;
    case 3:
      ffb->SetCondition((USB_FFBReport_SetCondition_Output_Data_t*) data, &gEffectStates[effectId]);
      //milos, added
      LogText("SetCond - cpOff:");
      LogBinary(&((USB_FFBReport_SetCondition_Output_Data_t*)data)->cpOffset, 2);
      LogText(", posC:");
      LogBinary(&((USB_FFBReport_SetCondition_Output_Data_t*)data)->positiveCoefficient, 2);
      LogText(", posS:");
      LogBinary(&((USB_FFBReport_SetCondition_Output_Data_t*)data)->positiveSaturation, 2);
      LogText(", deadB:");
      LogBinaryLf(&((USB_FFBReport_SetCondition_Output_Data_t*)data)->deadBand, 1);
      break;
    case 4:
      ffb->SetPeriodic((USB_FFBReport_SetPeriodic_Output_Data_t*) data, &gEffectStates[effectId]);
      //milos, added
      LogText("SetPer - mag:");
      LogBinary(&((USB_FFBReport_SetPeriodic_Output_Data_t*)data)->magnitude, 2);
      LogText(", off:");
      LogBinary(&((USB_FFBReport_SetPeriodic_Output_Data_t*)data)->offset, 2);
      LogText(", phs:");
      LogBinary(&((USB_FFBReport_SetPeriodic_Output_Data_t*)data)->phase, 1);
      LogText(", per:");
      LogBinaryLf(&((USB_FFBReport_SetPeriodic_Output_Data_t*)data)->period, 2);
      break;
    case 5:
      ffb->SetConstantForce((USB_FFBReport_SetConstantForce_Output_Data_t*) data, &gEffectStates[effectId]);
      //milos, added
      LogText("SetCF - mag:");
      LogBinaryLf(&((USB_FFBReport_SetConstantForce_Output_Data_t*)data)->magnitude, 2);
      break;
    case 6:
      ffb->SetRampForce((USB_FFBReport_SetRampForce_Output_Data_t*)data, &gEffectStates[effectId]);
      //milos, added
      LogText("SetRF - start:");
      LogBinary(&((USB_FFBReport_SetRampForce_Output_Data_t*)data)->rampStart, 1);
      LogText(", end:");
      LogBinaryLf(&((USB_FFBReport_SetRampForce_Output_Data_t*)data)->rampEnd, 1);
      break;
    case 7:
      //FfbHandle_SetCustomForceData((USB_FFBReport_SetCustomForceData_Output_Data_t*) data); //milos, commented since there was nothing implemented inside
      //LogTextLf("Set Custom Force Table");
      break;
    case 8:
      //FfbHandle_SetDownloadForceSample((USB_FFBReport_SetDownloadForceSample_Output_Data_t*) data); //milos, commented since there was nothing implemented inside
      //LogTextLf("Set download");
      break;
    case 9:
      break;
    case 10:
      FfbHandle_EffectOperation((USB_FFBReport_EffectOperation_Output_Data_t*) data);
      //milos, added
      LogText("SetEffOper - op:");
      LogBinary(&((USB_FFBReport_EffectOperation_Output_Data_t*)data)->operation, 1);
      LogText(", loop:");
      LogBinaryLf(&((USB_FFBReport_EffectOperation_Output_Data_t*)data)->loopCount, 1);
      break;
    case 11:
      FfbHandle_BlockFree((USB_FFBReport_BlockFree_Output_Data_t *) data);
      //milos, added
      LogText("SetBlockFree - id:");
      LogBinaryLf(&((USB_FFBReport_BlockFree_Output_Data_t *)data)->effectBlockIndex, 1);
      break;
    case 12:
      FfbHandle_DeviceControl((USB_FFBReport_DeviceControl_Output_Data_t*) data);
      LogText("SetDeviceControl:");
      LogBinaryLf(&((USB_FFBReport_DeviceControl_Output_Data_t*)data)->control, 1);
      break;
    case 13:
      FfbHandle_DeviceGain((USB_FFBReport_DeviceGain_Output_Data_t*) data);
      //milos, added
      LogText("SetDeviceGain:");
      LogBinaryLf(&((USB_FFBReport_DeviceGain_Output_Data_t*)data)->deviceGain, 1);
      break;
    case 14:
      //FfbHandle_SetCustomForce((USB_FFBReport_SetCustomForce_Output_Data_t*) data); //milos, commented since there was nothing implemented inside
      //LogTextLf("Set customforce");
      break;
    case 241:
      configHID((USB_ConfigReport*) data);
      break;
    default:
      break;
  };

  LEDs_SetAllLEDs(LEDS_NO_LEDS);
}

void FfbOnCreateNewEffect (USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, USB_FFBReport_PIDBlockLoad_Feature_Data_t *outData)
{
  outData->reportId = 6;
  outData->effectBlockIndex = GetNextFreeEffect();

  if (outData->effectBlockIndex == 0)
  {
    outData->loadStatus = 2;	// 1=Success,2=Full,3=Error
    LogText("Could not create effect");
  }
  else
  {
    gFFB.mAutoCenter = false;
    outData->loadStatus = 1;	// 1=Success,2=Full,3=Error

    volatile TEffectState* effect = &gEffectStates[outData->effectBlockIndex];

    effect->type = inData->effectType;
    effect->duration = USB_DURATION_INFINITE;
    effect->attackTime = 0x00; //milos, added
    effect->fadeTime = 0x00;
    effect->attackLevel = 0xFF;
    effect->fadeLevel = 0xFF;
    effect->startDelay = 0x00; //milos, added
    effect->direction = 0x00; //milos, added - 0deg
    effect->enableAxis = 0x00; //milos, added - enable X-axis
    effect->gain = 0x7FFF; //milos, changed from 0xFF since it is now 16bit (32767)
    effect->magnitude = 0; // milos, added
    effect->offset = 0x00;
    effect->phase = 0x00;
    effect->period = 0x3E8; //milos 1000ms (1Hz)

    ffb->CreateNewEffect(inData, effect);

    LogText("Created effect");
    LogBinary(&outData->effectBlockIndex, 1);
    LogText(", type");
    LogBinaryLf(&inData->effectType, 1);
  }
  outData->ramPoolAvailable = 0xFFFF;	// =0 or 0xFFFF - don't really know what this is used for?
  //	WaitMs(5);
}

void FfbHandle_SetEffect(USB_FFBReport_SetEffect_Output_Data_t *data)
{
  volatile TEffectState* effect = &gEffectStates[data->effectBlockIndex];
  ffb->SetEffect(data, effect);
  //LogTextLf("Set Effect"); //milos, commented
}

void FfbOnPIDPool(USB_FFBReport_PIDPool_Feature_Data_t *data)
{
  FreeAllEffects();

  data->reportId = 7;
  data->ramPoolSize = 0xFFFF;
  data->maxSimultaneousEffects = 0x0B;	// FFP supports playing up to 11 simultaneous effects
  data->memoryManagement = 3;
}

//milos commented these, since nothing was implemented inside
/*void FfbHandle_SetCustomForceData(USB_FFBReport_SetCustomForceData_Output_Data_t *data)
  {
  }

  void FfbHandle_SetDownloadForceSample(USB_FFBReport_SetDownloadForceSample_Output_Data_t *data)
  {
  LogTextLf("Download Force Samples");
  }*/

void FfbHandle_EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t *data)
{
  uint8_t eid = data->effectBlockIndex;

  if (eid == 0xFF)
    eid = 0x7F;	// All effects

  if (data->operation == 1)
  { // Start
    LogText("Start Effect - id:");
    LogBinaryLf(eid, 1);
    StartEffect(eid);
    if (!gDisabledEffects.effectId[eid])
      ffb->StartEffect(eid);
  }
  else if (data->operation == 2)
  { // StartSolo
    // Stop all first
    LogTextLf("Start Solo Effect");
    StopAllEffects();
    if (!gDisabledEffects.effectId[eid])
      ffb->StopEffect(0x7F); // TODO: wheel ?

    // Then start the given effect
    StartEffect(eid);

    if (!gDisabledEffects.effectId[eid])
      ffb->StartEffect(0x7F);	// TODO: wheel ?
  }
  else if (data->operation == 3)
  { // Stop
    LogText("Stop Effect - id:");
    LogBinaryLf(eid, 1);
    StopEffect(eid);
  }
}


void FfbHandle_BlockFree (USB_FFBReport_BlockFree_Output_Data_t *data)
{
  uint8_t eid = data->effectBlockIndex;

  if (eid == 0xFF)
  { // all effects
    FreeAllEffects();
    ffb->FreeEffect(0x7f); // TODO: does this work with the wheel?
  }
  else
  {
    FreeEffect(eid);
  }
}

#define DEVICE_PAUSED			  0x01
#define ACTUATORS_ENABLED		0x02
#define SAFETY_SWITCH			  0x04
#define ACTUATOR_OVERRIDE		0x08
#define ACTUATOR_POWER			0x10

void FfbHandle_DeviceControl(USB_FFBReport_DeviceControl_Output_Data_t *data)
{
  //	LogTextP(PSTR("Device Control: "));

  uint8_t control = data->control;
  // 1=Enable Actuators, 2=Disable Actuators, 3=Stop All Effects, 4=Reset, 5=Pause, 6=Continue

  // PID State Report:
  //	uint8_t	reportId;	// =2
  //	uint8_t	status;	// Bits: 0=Device Paused,1=Actuators Enabled,2=Safety Switch,3=Actuator Override Switch,4=Actuator Power
  //	uint8_t	effectBlockIndex;	// Bit7=Effect Playing, Bit0..7=EffectId (1..40)

  pidState.reportId = 2;
  Bset(pidState.status, SAFETY_SWITCH);
  Bset(pidState.status, ACTUATOR_POWER);
  pidState.effectBlockIndex = 0;

  switch (control)
  {
    case 0x01:
      LogTextLf("Disable Actuators");
      Bclr(pidState.status, ACTUATORS_ENABLED);
      break;
    case 0x02:
      LogTextLf("Enable Actuators");
      Bset(pidState.status, ACTUATORS_ENABLED);
      break;
    case 0x03:
      LogTextLf("Stop All Effects");		// Disable auto-center spring and stop all effects
      ffb->SetAutoCenter(0);
      pidState.effectBlockIndex = 0;
      break;
    case 0x04:
      LogTextLf("Reset");			// Reset (e.g. FFB-application out of focus)
      ffb->SetAutoCenter(1);		// Enable auto-center spring and stop all effects
      //		WaitMs(75);
      FreeAllEffects();
      break;
    case 0x05:
      LogTextLf("Pause");
      Bset(pidState.status, DEVICE_PAUSED);
      break;
    case 0x06:
      LogTextLf("Continue");
      Bclr(pidState.status, DEVICE_PAUSED);
      break;
    default:
      LogTextP(PSTR("Other "));
      LogBinaryLf(&data->control, 1);
  }
}

void FfbHandle_DeviceGain(USB_FFBReport_DeviceGain_Output_Data_t *data)
{
  /*
    uint8_t reportId; // =13
    uint8_t deviceGain; //0..255  (physical 0..10000) //milos, back to 8bit
  */
  //LogTextP(PSTR("Device Gain: "));
  //LogBinaryLf(&data->deviceGain, 1);
  //ffb->SetDeviceGain(data->deviceGain, 63); //milos, added
}

//milos, commented since nothing was implemented inside
/*void FfbHandle_SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t *data)
  {
  LogTextLf("Set Custom Force");
  //	LogBinary(&data, sizeof(USB_FFBReport_SetCustomForce_Output_Data_t));
  }*/

//------------------------------------------------------------------------------

void WaitMs(int ms)
{
  while (ms--)
    delay(1);
}

void FfbSendData(const uint8_t *data, uint16_t len)
{
}

void FfbSendPackets(const uint8_t *data, uint16_t len)
{
}

// ----------------------------------------------
// Debug and other settings
// ----------------------------------------------


// Send "enable FFB" to joystick
void FfbSendEnable()
{
}

// Send "disable FFB" to joystick
void FfbSendDisable()
{
}

/*
  typedef struct {
	uint8_t state;	// see constants <MEffectState_*>
	uint16_t usb_duration, usb_fadeTime;	// used to calculate fadeTime to MIDI, since in USB it is given as time difference from the end while in MIDI it is given as time from start
	// These are used to calculate effects of USB gain to MIDI data
	uint8_t usb_gain, usb_offset, usb_attackLevel, usb_fadeLevel;
	uint8_t usb_magnitude;
	FFP_MIDI_Effect_Basic	data;	// For FFP, this is enough for all types of effects - cast for other effect types when necessary
	} TEffectState;

  const uint8_t MEffectState_Allocated = 0x01;
  const uint8_t MEffectState_Playing = 0x02;
  const uint8_t MEffectState_SentToJoystick = 0x04;
*/

uint8_t FfbDebugListEffects(uint8_t *index)
{
  if (*index == 0)
    *index = 2;

  //	if (*index >= nextEID)
  if (*index >= MAX_EFFECTS)
    return 0;

  TEffectState *e = (TEffectState*) &gEffectStates[*index];
  LogBinary(index, 1);
  if (e->state == MEffectState_Allocated) {
    LogTextP(PSTR(" Allocated"));
  } else if (e->state == MEffectState_Playing) {
    LogTextP(PSTR(" Playing\n"));
  } else {
    LogTextP(PSTR(" Free"));
  }

  if (gDisabledEffects.effectId[*index]) {
    LogTextP(PSTR(" (Disabled)\n"));
  } else {
    LogTextP(PSTR(" (Enabled)\n"));
  }
  if (e->state) {
    LogTextP(PSTR("  duration="));
    LogBinary(&e->duration, 2);
    LogTextP(PSTR("\n  fadeTime="));
    LogBinary(&e->fadeTime, 2);
    LogTextP(PSTR("\n  gain="));
    LogBinary(&e->gain, 1);
  }

  *index = *index + 1;

  return 1;
}


void FfbEnableSprings(uint8_t inEnable)
{
  gDisabledEffects.springs = !inEnable;
}

void FfbEnableConstants(uint8_t inEnable)
{
  gDisabledEffects.constants = !inEnable;
}

void FfbEnableTriangles(uint8_t inEnable)
{
  gDisabledEffects.triangles = !inEnable;
}

void FfbEnableSines(uint8_t inEnable)
{
  gDisabledEffects.sines = !inEnable;
}

void FfbEnableEffectId(uint8_t inId, uint8_t inEnable)
{
  gDisabledEffects.effectId[inId] = !inEnable;

  if (gEffectStates[inId].state == MEffectState_Playing)
  {
    LogTextP(PSTR("Stop manual:"));
    LogBinaryLf(&inId, 1);
    StopEffect(inId);
  }
}

#endif
