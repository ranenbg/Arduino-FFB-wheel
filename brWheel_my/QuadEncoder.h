#ifndef _QUAD_ENCODER_H
#define _QUAD_ENCODER_H

#define QUAD_ENC_PIN_A		0
#define QUAD_ENC_PIN_B		1
#define QUAD_ENC_PIN_I		2

//Arduino UNO
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__)
#define CORE_PIN2_INT    0
#define CORE_PIN3_INT   1

// Arduino Mega
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define CORE_PIN2_INT   0
#define CORE_PIN3_INT   1
#define CORE_PIN18_INT    5
#define CORE_PIN19_INT    4
#define CORE_PIN20_INT    3
#define CORE_PIN21_INT    2

// Arduino Leonardo
#elif defined(__AVR_ATmega32U4__) && !defined(CORE_TEENSY)
#define CORE_PIN0_INT   2
#define CORE_PIN1_INT   3
#define CORE_PIN2_INT   1
#define CORE_PIN3_INT   0
#endif

//-----------------------------------------------------------------------------------------------

class cQuadEncoder {
  public:
    void Init (s32 position, b8 pullups = true); // milos, no additional pullup resistors needed if set true
    s32 Read ();
    void Write (s32 pos);
    void Update ();

  private:
    // 	volatile b8 mIndexFound;
    // 	volatile u8 mLastState;
    // 	volatile s32 mPosition;
};

extern cQuadEncoder gQuadEncoder;

#endif // _QUAD_ENCODER_H
