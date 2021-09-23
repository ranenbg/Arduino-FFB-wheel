#include "Config.h"
#include "debug.h"
#include "ConfigHID.h"


void configHID(USB_ConfigReport *data) {
  if (data->Info == 1) {
    int16_t rotation = data->Rotation;
    rotation = constrain(rotation, 30, 1800);
    ROTATION_MAX = (CPR / 360) * rotation;
    //brWheelFFB.offset = (MAX_ENCODER_ROTATION - ROTATION_MAX) / 2; //milos, commented
    //brWheelFFB.offset = 0; //milos, commented

    if (data->Calibrate == 1)
      brWheelFFB.calibrate();

    configGeneralGain = constrain(data->GeneralGain, 0, 200);
    configConstantGain = constrain(data->ConstantGain, 0, 200);
    configDamperGain = constrain(data->DamperGain, 0, 200);
    configFrictionGain = constrain(data->FrictionGain, 0, 200);
    configPeriodicGain = constrain(data->PeriodicGain, 0, 200);
    configSpringGain = constrain(data->SpringGain, 0, 200);
    configInertiaGain = constrain(data->InertiaGain, 0, 200);
    configCenterGain = constrain(data->CenterGain, 0, 200);
    configStopGain = constrain(data->StopGain, 0, 200);

    int16_t temp = data->MinForce;
    if (temp < MM_MAX_MOTOR_TORQUE)
    {
      MM_MIN_MOTOR_TORQUE = constrain(temp, 0, TOP - 1);
      data->Info = 99;
    }

    temp = data->MaxForce;
    if (temp > MM_MIN_MOTOR_TORQUE)
    {
      data->Info = 99;
      MM_MAX_MOTOR_TORQUE = constrain(temp, 1, TOP);
    }
  }
  else if (data->Info == 255) {
    /*uint8_t ReportId;
      uint16_t Rotation;
      uint8_t GeneralGain;
      uint8_t ConstantGain;
      uint8_t DamperGain;
      uint8_t FrictionGain;
      uint8_t PeriodicGain;
      uint8_t SpringGain;
      uint8_t InertiaGain;
      uint8_t CenterGain;
      uint8_t StopGain;
      uint16_t MaxForce;
      uint16_t MinForce;
      uint8_t Centralize;
      uint8_t Calibrate;
      uint8_t Info;
      uint8_t Version;*/
    data->Rotation = ROTATION_MAX / CPR * 360;
    data->GeneralGain = configGeneralGain;
    data->ConstantGain = configConstantGain;
    data->DamperGain = configDamperGain;
    data->FrictionGain = configFrictionGain;
    data->PeriodicGain = configPeriodicGain;
    data->SpringGain = configSpringGain;
    data->InertiaGain = configInertiaGain;
    data->CenterGain = configCenterGain;
    data->StopGain = configStopGain;
    data->MaxForce = MM_MAX_MOTOR_TORQUE;
    data->MinForce = MM_MIN_MOTOR_TORQUE;
  }
  data->Version = VERSION;
  //uint8_t *response = (uint8_t*)data;
  HID_SendReport(0xF2, (uint8_t*)data, 64);
}

