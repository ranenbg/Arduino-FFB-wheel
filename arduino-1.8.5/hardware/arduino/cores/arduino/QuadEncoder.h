#ifndef _QUAD_ENCODER_H
#define _QUAD_ENCODER_H

#define QUAD_ENC_PIN_A		0
#define QUAD_ENC_PIN_B		1
#define QUAD_ENC_PIN_I		2

s16 ROTATION_DEG; //milos
s32 CPR; //milos
s32 ROTATION_MAX; //milos
s32 ROTATION_MID; // milos

//-----------------------------------------------------------------------------------------------

class cQuadEncoder
{
  public:
    void Init (s32 position, b8 pullups = true); // milos, no additional pullup resistors needed if set true
    int32_t Read ();
    void Write (s32 pos);
    void Update ();

  private:
    // 	volatile b8 mIndexFound;
    // 	volatile u8 mLastState;
    // 	volatile s32 mPosition;
};

extern cQuadEncoder gQuadEncoder;

#endif // _QUAD_ENCODER_H
