

/* Copyright (c) 2011, Peter Barrett
**
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

#include "Platform.h"
#include "USBAPI.h"
#include "USBDesc.h"
#include "HID.h"

#if defined(USBCON)
#ifdef HID_ENABLED

//#define RAWHID_ENABLED

Mouse_ Mouse;
Keyboard_ Keyboard;
Joystick_ Joystick;

//================================================================================
#define RAWHID_USAGE_PAGE	0xFFC0
#define RAWHID_USAGE		0x0C00
#define RAWHID_TX_SIZE 64
#define RAWHID_RX_SIZE 64

//#define ADD_KEYBOARD
//#define ADD_MOUSE
#define ADD_JOYSTICK

extern const u8 _hidReportDescriptor[] PROGMEM;
const u8 _hidReportDescriptor[] =
{
#ifdef ADD_MOUSE
  //	Mouse
  0x05, 0x01,                   // USAGE_PAGE (Generic Desktop)	// 54
  0x09, 0x02,                   // USAGE (Mouse)
  0xa1, 0x01,                   // COLLECTION (Application)
  0x09, 0x01,                   //   USAGE (Pointer)
  0xa1, 0x00,                   //   COLLECTION (Physical)
  0x85, 0x01,                   //     REPORT_ID (1)
  0x05, 0x09,                   //     USAGE_PAGE (Button)
  0x19, 0x01,                   //     USAGE_MINIMUM (Button 1)
  0x29, 0x03,                   //     USAGE_MAXIMUM (Button 3)
  0x15, 0x00,                   //     LOGICAL_MINIMUM (0)
  0x25, 0x01,                   //     LOGICAL_MAXIMUM (1)
  0x95, 0x03,                   //     REPORT_COUNT (3)
  0x75, 0x01,                   //     REPORT_SIZE (1)
  0x81, 0x02,                   //     INPUT (Data,Var,Abs)
  0x95, 0x01,                   //     REPORT_COUNT (1)
  0x75, 0x05,                   //     REPORT_SIZE (5)
  0x81, 0x03,                   //     INPUT (Cnst,Var,Abs)
  0x05, 0x01,                   //     USAGE_PAGE (Generic Desktop)
  0x09, 0x30,                   //     USAGE (X)
  0x09, 0x31,                   //     USAGE (Y)
  0x09, 0x38,                   //     USAGE (Wheel)
  0x15, 0x81,                   //     LOGICAL_MINIMUM (-127)
  0x25, 0x7f,                   //     LOGICAL_MAXIMUM (127)
  0x75, 0x08,                   //     REPORT_SIZE (8)
  0x95, 0x03,                   //     REPORT_COUNT (3)
  0x81, 0x06,                   //     INPUT (Data,Var,Rel)
  0xc0,                          //   END_COLLECTION
  0xc0,                          // END_COLLECTION
#endif
#ifdef ADD_KEYBOARD
  //	Keyboard
  0x05, 0x01,                   // USAGE_PAGE (Generic Desktop)	// 47
  0x09, 0x06,                   // USAGE (Keyboard)
  0xa1, 0x01,                   // COLLECTION (Application)
  0x85, 0x02,                   //   REPORT_ID (2)
  0x05, 0x07,                   //   USAGE_PAGE (Keyboard)

  0x19, 0xe0,                   //   USAGE_MINIMUM (Keyboard LeftControl)
  0x29, 0xe7,                   //   USAGE_MAXIMUM (Keyboard Right GUI)
  0x15, 0x00,                   //   LOGICAL_MINIMUM (0)
  0x25, 0x01,                   //   LOGICAL_MAXIMUM (1)
  0x75, 0x01,                   //   REPORT_SIZE (1)

  0x95, 0x08,                   //   REPORT_COUNT (8)
  0x81, 0x02,                   //   INPUT (Data,Var,Abs)
  0x95, 0x01,                   //   REPORT_COUNT (1)
  0x75, 0x08,                   //   REPORT_SIZE (8)
  0x81, 0x03,                   //   INPUT (Cnst,Var,Abs)

  0x95, 0x06,                   //   REPORT_COUNT (6)
  0x75, 0x08,                   //   REPORT_SIZE (8)
  0x15, 0x00,                   //   LOGICAL_MINIMUM (0)
  0x25, 0x65,                   //   LOGICAL_MAXIMUM (101)
  0x05, 0x07,                   //   USAGE_PAGE (Keyboard)

  0x19, 0x00,                   //   USAGE_MINIMUM (Reserved (no event indicated))
  0x29, 0x65,                   //   USAGE_MAXIMUM (Keyboard Application)
  0x81, 0x00,                   //   INPUT (Data,Ary,Abs)
  0xc0,                          // END_COLLECTION
#endif
#if RAWHID_ENABLED
  //	RAW HID
  0x06, LSB(RAWHID_USAGE_PAGE), MSB(RAWHID_USAGE_PAGE),	// 30
  0x0A, LSB(RAWHID_USAGE), MSB(RAWHID_USAGE),

  0xA1, 0x01,				// Collection 0x01
  0x85, 0x03,            // REPORT_ID (3)
  0x75, 0x08,				// report size = 8 bits
  0x15, 0x00,				// logical minimum = 0
  0x26, 0xFF, 0x00,		// logical maximum = 255

  0x95, 64,				// report count TX
  0x09, 0x01,				// usage
  0x81, 0x02,				// Input (array)

  0x95, 64,				// report count RX
  0x09, 0x02,				// usage
  0x91, 0x02,				// Output (array)
  0xC0					// end collection
#endif
#ifdef ADD_JOYSTICK
  0x05, 0x01,	// USAGE_PAGE (Generic Desktop)
  0x09, 0x04,	// USAGE (Joystick)
  0xA1, 0x01,	// COLLECTION (Application)
  0x85, 0x04,	// REPORT_ID (04)
  0x09, 0x01, // USAGE (Pointer)
  //0x05, 0x01,							/*   USAGE_PAGE (Generic Desktop) */
  0xA1, 0x00, // COLLECTION (Physical)

  0x09, 0x30,          // USAGE (x)
  //0x16, X_AXIS_LOG_MIN & 0xFF, (X_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM // milos, old
  //0x27, X_AXIS_LOG_MAX & 0xFF, (X_AXIS_LOG_MAX >> 8) & 0xFF, 0, 0, // LOGICAL_MAXIMUM // milos, old
  0x17, X_AXIS_LOG_MIN & 0xFF, (X_AXIS_LOG_MIN >> 8) & 0xFF, (X_AXIS_LOG_MIN >> 16) & 0xFF, (X_AXIS_LOG_MIN >> 24) & 0xFF, // LOGICAL_MINIMUM // milos, new 32bit
  0x27, X_AXIS_LOG_MAX & 0xFF, (X_AXIS_LOG_MAX >> 8) & 0xFF, (X_AXIS_LOG_MAX >> 16) & 0xFF, (X_AXIS_LOG_MAX >> 24) & 0xFF, // LOGICAL_MAXIMUM // milos, new 32bit
  0x35, 0x00,         // PHYSICAL_MINIMUM (00)
  0x47, X_AXIS_PHYS_MAX & 0xFF, (X_AXIS_PHYS_MAX >> 8) & 0xFF, (X_AXIS_PHYS_MAX >> 16) & 0xFF, (X_AXIS_PHYS_MAX >> 24) & 0xFF, // PHYSICAL_MAXIMUM (0xffff) // milos, new 32bits
  0x75, X_AXIS_NB_BITS,   // REPORT_SIZE (AXIS_NB_BITS)
  0x95, 0x01,            // REPORT_COUNT (1)
  0x81, 0x02,         // INPUT (Data,Var,Abs)

  0x09, 0x31,         // USAGE (y)
  0x16, Y_AXIS_LOG_MIN & 0xFF, (Y_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
  0x27, Y_AXIS_LOG_MAX & 0xFF, (Y_AXIS_LOG_MAX >> 8) & 0xFF, 0, 0, // LOGICAL_MAXIMUM
  0x35, 0x00,         // PHYSICAL_MINIMUM (00)
  0x47, Y_AXIS_PHYS_MAX & 0xFF, (Y_AXIS_PHYS_MAX >> 8) & 0xFF, 0, 0,//(X_AXIS_PHYS_MAX >> 16) & 0xFF,(X_AXIS_PHYS_MAX >> 24) & 0xFF, // PHYSICAL_MAXIMUM (0xffff)
  0x75, Y_AXIS_NB_BITS,   // REPORT_SIZE (AXIS_NB_BITS)
  0x95, 0x01,           // REPORT_COUNT (1)
  0x81, 0x02,         // INPUT (Data,Var,Abs)

  0x09, 0x32,         // USAGE (z)
  0x16, Z_AXIS_LOG_MIN & 0xFF, (Z_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
  0x27, Z_AXIS_LOG_MAX & 0xFF, (Z_AXIS_LOG_MAX >> 8) & 0xFF, 0, 0, // LOGICAL_MAXIMUM
  0x35, 0x00,         // PHYSICAL_MINIMUM (00)
  0x47, Z_AXIS_PHYS_MAX & 0xFF, (Z_AXIS_PHYS_MAX >> 8) & 0xFF, 0, 0,//(Z_AXIS_PHYS_MAX >> 16) & 0xFF,(Z_AXIS_PHYS_MAX >> 24) & 0xFF, // PHYSICAL_MAXIMUM (0xffff)
  0x75, Z_AXIS_NB_BITS,   // REPORT_SIZE (AXIS_NB_BITS)
  0x95, 0x01,           // REPORT_COUNT (1)
  0x81, 0x02,         // INPUT (Data,Var,Abs)

  0x09, 0x33,         // USAGE (rx)
  0x16, RX_AXIS_LOG_MIN & 0xFF, (RX_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
  0x27, RX_AXIS_LOG_MAX & 0xFF, (RX_AXIS_LOG_MAX >> 8) & 0xFF, 0, 0, // LOGICAL_MAXIMUM
  0x35, 0x00,         // PHYSICAL_MINIMUM (00)
  0x47, RX_AXIS_PHYS_MAX & 0xFF, (RX_AXIS_PHYS_MAX >> 8) & 0xFF, 0, 0,//(RX_AXIS_PHYS_MAX >> 16) & 0xFF,(RX_AXIS_PHYS_MAX >> 24) & 0xFF, // PHYSICAL_MAXIMUM (0xffff)
  0x75, RX_AXIS_NB_BITS,   // REPORT_SIZE (AXIS_NB_BITS)
  0x95, 0x01,           // REPORT_COUNT (1)
  0x81, 0x02,         // INPUT (Data,Var,Abs)

  0x09, 0x34,         // USAGE (ry)
  0x16, RY_AXIS_LOG_MIN & 0xFF, (RY_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
  0x27, RY_AXIS_LOG_MAX & 0xFF, (RY_AXIS_LOG_MAX >> 8) & 0xFF, 0, 0, // LOGICAL_MAXIMUM
  0x35, 0x00,         // PHYSICAL_MINIMUM (00)
  0x47, RY_AXIS_PHYS_MAX & 0xFF, (RY_AXIS_PHYS_MAX >> 8) & 0xFF, 0, 0,//(RY_AXIS_PHYS_MAX >> 16) & 0xFF,(RY_AXIS_PHYS_MAX >> 24) & 0xFF, // PHYSICAL_MAXIMUM (0xffff)
  0x75, RY_AXIS_NB_BITS,   // REPORT_SIZE (AXIS_NB_BITS)
  0x95, 0x01,           // REPORT_COUNT (1)
  0x81, 0x02,         // INPUT (Data,Var,Abs)

  /*0x09, 0x35,         // USAGE (rz)
    0x16, RZ_AXIS_LOG_MIN & 0xFF, (RZ_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
    0x27, RZ_AXIS_LOG_MAX & 0xFF, (RZ_AXIS_LOG_MAX >> 8) & 0xFF, 0, 0, // LOGICAL_MAXIMUM
    0x35, 0x00,         // PHYSICAL_MINIMUM (00)
    0x47, RZ_AXIS_PHYS_MAX & 0xFF, (RZ_AXIS_PHYS_MAX >> 8) & 0xFF, 0, 0,//(RZ_AXIS_PHYS_MAX >> 16) & 0xFF,(RZ_AXIS_PHYS_MAX >> 24) & 0xFF, // PHYSICAL_MAXIMUM (0xffff)
    0x75, RZ_AXIS_NB_BITS,   // REPORT_SIZE (AXIS_NB_BITS)
    0x95, 0x01,           // REPORT_COUNT (1)
    0x81, 0x02,         // INPUT (Data,Var,Abs)*/

  //0xc0, // END_COLLECTION

  0x09, 0x39,                     // USAGE (HAT SWITCH)
  0x15, 0x01,                     // LOGICAL_MINIMUM (1)
  0x25, 0x08,                     // LOGICAL_MAXIMUM (8)
  0x35, 0x00,                     // PHYSICAL_MINIMUM (0)
  0x46, 0x3B, 0x01,               // PHYSICAL_MAXIMUM (315)
  0x65, 0x14,                     // UNIT (Eng Rot:Angular Pos)
  0x55, 0x00,                     // UNIT_EXPONENT (0)
  0x75, 0x04,                     // REPORT_SIZE (4)
  0x95, 0x01,                     // REPORT_COUNT (1)
  0x81, 0x02,                     // Input (Data,Var,Abs)

  0x05, 0x09,                     // USAGE_PAGE (Button)
  0x15, 0x00,                     // LOGICAL_MINIMUM (0)
  0x25, 0x01,                     // LOGICAL_MAXIMUM (1)
  0x55, 0x00,                     // UNIT_EXPONENT (0)
  0x65, 0x00,                     // UNIT (None)
  0x19, 0x01,					            // USAGE_MINIMUM (button 1)
  0x29, NB_BUTTONS,               // USAGE_MAXIMUM (button NB_BUTTONS)
  0x75, 0x01,                     // REPORT_SIZE (1)
  0x95, NB_BUTTONS,			          // REPORT_COUNT (NB_BUTTONS)
  0x81, 0x02,                     // Input (Data,Var,Abs)

  // FOR CONFIG PROFILE
  0x85, 0xf1,                    //   REPORT_ID (f1)
  0x09, 0x01,                    //   USAGE (Vendor Usage 1)
  0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
  0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
  0x95, 0x3F, //0x20,                    //   REPORT_COUNT (32)
  0x75, 0x08,                    //   REPORT_SIZE (8)
  0x91, 0x82,                    //   OUTPUT (Data,Var,Abs,Vol)	//8

  0x85, 0xf2,                    //   REPORT_ID (f2)
  0x09, 0x01,                    //   USAGE (Vendor Usage 3)
  0x95, 0x3F, //0x20,                    //   REPORT_COUNT (32)
  0x75, 0x08,                    //   REPORT_SIZE (8)
  0x81, 0x82,                    //   INPUT (Data,Var,Abs,Vol)	8
  0xc0, // END_COLLECTION


  //FFB part (PID) starts from here
  0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
  0x09, 0x92,	// USAGE (PID State Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x02,	// REPORT_ID (02)
  0x09, 0x9F,	// USAGE (Device Paused)
  0x09, 0xA0,	// USAGE (Actuators Enabled)
  0x09, 0xA4,	// USAGE (Safety Switch)
  0x09, 0xA5,	// USAGE (Actuator Override Switch)
  0x09, 0xA6,	// USAGE (Actuator Power)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x25, 0x01,	// LOGICAL_MINIMUM (01)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x45, 0x01,	// PHYSICAL_MAXIMUM (01)
  0x75, 0x01,	// REPORT_SIZE (01)
  0x95, 0x05,	// REPORT_COUNT (05)
  0x81, 0x02,	// INPUT (Data,Var,Abs)
  0x95, 0x03,	// REPORT_COUNT (03)
  0x81, 0x03,	// INPUT (Constant,Var,Abs)
  0x09, 0x94,	// USAGE (Effect Playing)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x25, 0x01,	// LOGICAL_MAXIMUM (01)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x45, 0x01,	// PHYSICAL_MAXIMUM (01)
  0x75, 0x01,	// REPORT_SIZE (01)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x81, 0x02,	// INPUT (Data,Var,Abs)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x07,	// REPORT_SIZE (07)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x81, 0x02,	// INPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  0x09, 0x21,	// USAGE (Set Effect Output Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x01,	// REPORT_ID (01)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x25,	// USAGE (Effect type)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x09, 0x26,	// USAGE (ET Constant Force)
  0x09, 0x27,	// USAGE (ET Ramp)
  0x09, 0x30,	// USAGE (ET Square)
  0x09, 0x31,	// USAGE (ET Sine)
  0x09, 0x32,	// USAGE (ET Triangle)
  0x09, 0x33,	// USAGE (ET Sawtooth Up)
  0x09, 0x34,	// USAGE (ET Sawtooth Down)
  0x09, 0x40,	// USAGE (ET Spring)
  0x09, 0x41,	// USAGE (ET Damper)
  0x09, 0x42,	// USAGE (ET Inertia)
  0x09, 0x43,	// USAGE (ET Friction)
  //0x09, 0x28,	// USAGE (ET Custom Force Data) //milos, removed custom force effect block
  //0x25, 0x0C,	// LOGICAL_MAXIMUM (12)
  0x25, 0x0B,  // LOGICAL_MAXIMUM (11) //milos, 1 less effect
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  //0x45, 0x0C,	// PHYSICAL_MAXIMUM (12)
  0x45, 0x0B,  // PHYSICAL_MAXIMUM (11) //milos, 1 less effect
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x00,	// OUTPUT (Data)
  0xC0,	// END COLLECTION ()
  0x09, 0x50,	// USAGE (Duration)
  0x09, 0x54,	// USAGE (Trigger Repeat Interval)
  //0x09, 0x51,	// USAGE (Sample Period) //milos, commented
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x27, 0xFF, 0xFF, 0x00, 0x00,	// LOGICAL_MAXIMUM (65535)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x47, 0xFF, 0xFF, 0x00, 0x00,	// PHYSICAL_MAXIMUM (65535)
  0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
  0x55, 0xFD,  // UNIT_EXPONENT (-3)
  0x75, 0x10,	// REPORT_SIZE (16)
  //0x95, 0x03,	// REPORT_COUNT (03)
  0x95, 0x02,  // REPORT_COUNT (02) //milos
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x55, 0x00,	// UNIT_EXPONENT (00)
  0x66, 0x00, 0x00,	// UNIT (None)
  0x09, 0x52,	// USAGE (Gain)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  //0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
  0x26, 0xFF, 0x7F, // LOGICAL_MAXIMUM (32767) //milos
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  //0x46, 0x10, 0x27,	// PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  //0x75, 0x08,	// REPORT_SIZE (08)
  0x75, 0x10,  // REPORT_SIZE (16) //milos
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x53,	// USAGE (Trigger Button)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x08,	// LOGICAL_MAXIMUM (08)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x08,	// PHYSICAL_MAXIMUM (08)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x55,	// USAGE (Axes Enable)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x05, 0x01,	// USAGE_PAGE (Generic Desktop)
  0x09, 0x30,	// USAGE (X)
#ifdef NB_FF_AXIS>1
  0x09, 0x31,	// USAGE (Y)
#endif
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x25, 0x01,	// LOGICAL_MAXIMUM (01)
  0x75, 0x01,	// REPORT_SIZE (01)
  0x95, NB_FF_AXIS,	// REPORT_COUNT (NB_FF_AXIS)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
  0x09, 0x56,	// USAGE (Direction Enable)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x95, 0x07 - NB_FF_AXIS,	// REPORT_COUNT (05 (2 axes) or 06 (1 axes)) seems to be for padding
  0x91, 0x03,	// OUTPUT (Constant,Var,Abs)
  0x09, 0x57,	// USAGE (Direction)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x0B, 0x01, 0x00, 0x0A, 0x00, // USAGE (Ordinals:Instance 1)
  0x0B, 0x02, 0x00, 0x0A, 0x00, // USAGE (Ordinals:Instance 2)
  0x66, 0x14, 0x00,	// UNIT (Eng Rot:Angular Pos)
  0x55, 0xFE,  // UNIT_EXPONENT (-2) //milos, 16bit
  //0x55, 0x00, // UNIT_EXPONENT (0) //milos, 8bit
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x26, 0xFF, 0x7F,  // LOGICAL_MAXIMUM (32767) //milos, 16bit
  //0x26, 0xFF, 0x00,  // LOGICAL_MAXIMUM (255) //milos, 8bit
  0x35, 0x00,	// PHYSICAL_MINIMUM (0)
  0x47, 0x9F, 0x8C, 0x00, 0x00,	// PHYSICAL_MAXIMUM (35999) //milos, 16bit
  //0x46, 0x67, 0x01,  // PHYSICAL_MAXIMUM (359) //milos, 8bit
  0x75, 0x10,  // REPORT_SIZE (16) //milos, 16bit
  //0x75, 0x08,  // REPORT_SIZE (08) //milos, 8bit
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x55, 0x00,	// UNIT_EXPONENT (00)
  0x66, 0x00, 0x00,	// UNIT (None)
  0xC0,	// END COLLECTION ()
  0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
  0x09, 0xA7,	// USAGE (Start Delay) //milos, uncommented
  0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
  0x55, 0xFD,  // UNIT_EXPONENT (-3)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x27, 0xFF, 0xFF, 0x00, 0x00,	// LOGICAL_MAXIMUM (65535)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x47, 0xFF, 0xFF, 0x00, 0x00,	// PHYSICAL_MAXIMUM (65535)
  0x75, 0x10,	// REPORT_SIZE (16)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs) //milos, uncommented
  0x66, 0x00, 0x00,	// UNIT (None)
  0x55, 0x00,	// UNIT_EXPONENT (00)
  0xC0,	// END COLLECTION ()

  0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
  0x09, 0x5A,	// USAGE (Set Envelope Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x02,	// REPORT_ID (02)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x5B,	// USAGE (Attack Level)
  0x09, 0x5D,	// USAGE (Fade Level)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x26, 0xFF, 0x00,  // LOGICAL_MAXIMUM (255)
  //0x26, 0xFF, 0x7F,	// LOGICAL_MAXIMUM (32767) //milos
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  0x75, 0x08, // REPORT_SIZE (08)
  0x95, 0x02,	// REPORT_COUNT (02)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x5C,	// USAGE (Attack Time)
  0x09, 0x5E,	// USAGE (Fade Time)
  0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
  0x55, 0xFD,  // UNIT_EXPONENT (-3)
  //0x26, 0xFF, 0xFF,	// LOGICAL_MAXIMUM (65535)
  //0x46, 0xFF, 0xFF,	// PHYSICAL_MAXIMUM (65535)
  0x26, 0xFF, 0x7F, // LOGICAL_MAXIMUM (32767) //milos
  0x46, 0xFF, 0x7F, // PHYSICAL_MAXIMUM (32767) //milos
  0x75, 0x10,	// REPORT_SIZE (16)
  0x95, 0x02, // REPORT_COUNT (02) //milos, added
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x66, 0x00, 0x00,	// UNIT (None)
  0x55, 0x00,	// UNIT_EXPONENT (00)
  0xC0,	// END COLLECTION ()

  0x09, 0x5F,	// USAGE (Set Condition Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x03,	// REPORT_ID (03)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x23,	// USAGE (Parameter Block Offset)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x25, 0x01,	// LOGICAL_MAXIMUM (01)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x45, 0x01,	// PHYSICAL_MAXIMUM (01)
  0x75, 0x04,	// REPORT_SIZE (04)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x58,	// USAGE (Type Specific Block Offset)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x0B, 0x01, 0x00, 0x0A, 0x00,	// USAGE (Instance 1)
  0x0B, 0x02, 0x00, 0x0A, 0x00,	// USAGE (Instance 2)
  0x75, 0x02,	// REPORT_SIZE (02)
  0x95, 0x02,	// REPORT_COUNT (02)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()
  0x09, 0x60, // USAGE (CP Offset)
  //0x15, 0x80,	// LOGICAL_MINIMUM (-128)
  0x16, 0x00, 0x80,  // LOGICAL_MINIMUM (-32768) //milos
  //0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
  0x26, 0xFF, 0x7F,  // LOGICAL_MAXIMUM (32767) //milos
  //0x36, 0xF0, 0xD8,  // PHYSICAL_MINIMUM (-10000)
  0x36, 0x00, 0x80,  // PHYSICAL_MINIMUM (-32768) //milos
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  //0x75, 0x08,	// REPORT_SIZE (08)
  0x75, 0x10, // REPORT_SIZE (16) //milos
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x61, // USAGE (Positive Coefficient)
  //   0x09,0x62,  // USAGE (Negative Coefficient)
  //0x36, 0xF0, 0xD8,	// PHYSICAL_MINIMUM (-10000)
  0x36, 0x00, 0x80,  // PHYSICAL_MINIMUM (-32768) //milos
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  0x95, 0x01,	// REPORT_COUNT (01)	// ???? WAS 2 with "negative coeff"
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  //0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
  0x26, 0xFF, 0x7F,  // LOGICAL_MAXIMUM (32767) //milos
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  0x09, 0x63,	// USAGE (Positive Saturation) //milos, uncommented
  //  0x09, 0x64,	// USAGE (Negative Saturation)
  0x75, 0x10,	// REPORT_SIZE (16) //milos
  0x95, 0x01,	// REPORT_COUNT (01) //milos, uncommented
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x65,	// USAGE (Dead Band ) //milos, uncommented
  0x15, 0x00,  // LOGICAL_MINIMUM (00) //milos
  0x26, 0xFF, 0x00, // LOGICAL_MAXIMUM (255) //milos
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  0x75, 0x08, // REPORT_SIZE (08) //milos
  0x95, 0x01,	// REPORT_COUNT (01) //milos, uncommented
  0x91, 0x02,	// OUTPUT (Data,Var,Abs) //milos, uncommented
  0xC0,	// END COLLECTION ()

  0x09, 0x6E,	// USAGE (Set Periodic Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x04,	// REPORT_ID (04)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x70,	// USAGE (Magnitude)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  //0x26, 0xFF, 0x00,   // LOGICAL_MAXIMUM (255)
  0x26, 0xFF, 0x7F,	// LOGICAL_MAXIMUM (32767) //milos
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F, // PHYSICAL_MAXIMUM (32767) //milos
  //0x75, 0x08,	// REPORT_SIZE (08)
  0x75, 0x10,  // REPORT_SIZE (16) //milos
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x6F,	// USAGE (Offset)
  //0x15, 0x80,	// LOGICAL_MINIMUM (-128)
  0x16, 0x00, 0x80,  // LOGICAL_MINIMUM (-32768) //milos
  //0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
  0x26, 0xFF, 0x7F,   // LOGICAL_MAXIMUM (32737) //milos
  //0x36, 0xF0, 0xD8,  // PHYSICAL_MINIMUM (-10000)
  0x36, 0x00, 0x80,  // PHYSICAL_MINIMUM (-32768) //milos
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x71,	// USAGE (Phase)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
  //0x27, 0xFF, 0xFF, 0x00, 0x00,  // LOGICAL_MAXIMUM (65535) //milos
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  //0x47, 0x9F, 0x8C, 0x00, 0x00,	// PHYSICAL_MAXIMUM (35999) //milos
  0x46, 0x67, 0x01,  // PHYSICAL_MAXIMUM (359) //milos
  0x66, 0x14, 0x00,  // UNIT (Eng Rot:Angular Pos)
  //0x55, 0xFE, // UNIT_EXPONENT (-2)
  0x55, 0x00, // UNIT_EXPONENT (0) //milos
  0x75, 0x08, // REPORT_SIZE (08) //milos
  0x95, 0x01, // REPORT_COUNT (01) //milos
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x72,	// USAGE (Period)
  //0x26, 0xFF, 0x7F,	// LOGICAL_MAXIMUM (32767)
  0x27, 0xFF, 0xFF, 0x00, 0x00,  // LOGICAL_MAXIMUM (65535) //milos
  //0x46, 0xFF, 0x7F,	// PHYSICAL_MAXIMUM (32767)
  0x47, 0xFF, 0xFF, 0x00, 0x00, // PHYSICAL_MAXIMUM (65535) //milos
  0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
  0x55, 0xFD,  // UNIT_EXPONENT (-3)
  0x75, 0x10,	// REPORT_SIZE (16)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x66, 0x00, 0x00,	// UNIT (None)
  0x55, 0x00,	// UNIT_EXPONENT (00)
  0xC0,	// END COLLECTION ()

  0x09, 0x73,	// USAGE (Set Constant Force Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x05,	// REPORT_ID (05)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x70,	// USAGE (Magnitude)
  //0x16, 0x01, 0xFF,	// LOGICAL_MINIMUM (-255)
  0x16, 0x01, 0x80,  // LOGICAL_MINIMUM (-32767) //milos, my original
  //0x16, 0xF0, 0xD8,  // LOGICAL_MINIMUM (-10000) //milos, testing
  //0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
  0x26, 0xFF, 0x7F, // LOGICAL_MAXIMUM (32767) //milos, my original
  //0x26, 0x10, 0x27, // LOGICAL_MAXIMUM (10000) //milos, testing
  //0x36, 0xF0, 0xD8,  // PHYSICAL_MINIMUM (-10000) //milos, testing
  0x36, 0x01, 0x80,  // PHYSICAL_MINIMUM (-32767) //milos, my original
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000) //milos, testing
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos, my original
  0x75, 0x10,	// REPORT_SIZE (16)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  0x09, 0x74,	// USAGE (Set Ramp Force Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x06,	// REPORT_ID (06)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x75,	// USAGE (Ramp Start)
  0x09, 0x76,	// USAGE (Ramp End)
  0x15, 0x81,	// LOGICAL_MINIMUM (-127)
  0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
  //0x36, 0xF0, 0xD8,  // PHYSICAL_MINIMUM (-10000)
  0x36, 0x01, 0x80,  // PHYSICAL_MINIMUM (-32767) //milos
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x02,	// REPORT_COUNT (02)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  //milos, commented since it was not used
  /*0x09, 0x68,	// USAGE (Custom Force Data Report)
    0xA1, 0x02,	// COLLECTION (Logical)
    0x85, 0x07,	// REPORT_ID (07)
    0x09, 0x22,	// USAGE (Effect Block Index)
    0x15, 0x01,	// LOGICAL_MINIMUM (01)
    0x25, 0x28,	// LOGICAL_MAXIMUM (40)
    0x35, 0x01,	// PHYSICAL_MINIMUM (01)
    0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
    0x75, 0x08,	// REPORT_SIZE (08)
    0x95, 0x01,	// REPORT_COUNT (01)
    0x91, 0x02,	// OUTPUT (Data,Var,Abs)
    0x09, 0x6C,	// USAGE (Custom Force Data Offset)
    //0x15, 0x00,	// LOGICAL_MINIMUM (00)
    0x16, 0x00, 0x80,  // LOGICAL_MINIMUM (-32768) //milos
    //0x26, 0x10, 0x27,	// LOGICAL_MAXIMUM (10000)
    0x26, 0xFF, 0x7F,  // LOGICAL_MAXIMUM (32767) //milos
    //0x35, 0x00,	// PHYSICAL_MINIMUM (00)
    0x36, 0x00, 0x80,  // PHYSICAL_MINIMUM (-32768) //milos
    //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
    0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
    0x75, 0x10,	// REPORT_SIZE (16)
    0x95, 0x01,	// REPORT_COUNT (01)
    0x91, 0x02,	// OUTPUT (Data,Var,Abs)
    0x09, 0x69,	// USAGE (Custom Force Data)
    0x15, 0x81,	// LOGICAL_MINIMUM (-127)
    0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
    //0x35, 0x00,	// PHYSICAL_MINIMUM (00)
    0x36, 0x01, 0x80,  // PHYSICAL_MINIMUM (-32767) //milos
    //0x46, 0xFF, 0x00,	// PHYSICAL_MAXIMUM (255)
    0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
    0x75, 0x08,	// REPORT_SIZE (08)
    0x95, 0x0C,	// REPORT_COUNT (12)
    0x92, 0x02, 0x01,	// OUTPUT (Data,Var,Abs,Buf)
    0xC0,	// END COLLECTION ()*/

  //milos, commented since it was not used
  /*0x09, 0x66,	// USAGE (Download Force Sample)
    0xA1, 0x02,	// COLLECTION (Logical)
    0x85, 0x08,	// REPORT_ID (08)
    0x05, 0x01,	// USAGE_PAGE (Generic Desktop)
    0x09, 0x30,	// USAGE (X)
    0x09, 0x31,	// USAGE (Y)
    0x15, 0x81,	// LOGICAL_MINIMUM (-127)
    0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
    //0x35, 0x00, // PHYSICAL_MINIMUM (00)
    0x36, 0x01, 0x80,  // PHYSICAL_MINIMUM (-32767) //milos
    //0x46, 0xFF, 0x00, // PHYSICAL_MAXIMUM (255)
    0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
    0x75, 0x08,	// REPORT_SIZE (08)
    0x95, 0x02,	// REPORT_COUNT (02)
    0x91, 0x02,	// OUTPUT (Data,Var,Abs)
    0xC0,	// END COLLECTION ()*/

  0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
  0x09, 0x77,	// USAGE (Effect Operation Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x0A,	// REPORT_ID (10)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x78,	// USAGE (78)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x09, 0x79,	// USAGE (Op Effect Start)
  0x09, 0x7A,	// USAGE (Op Effect Start Solo)
  0x09, 0x7B,	// USAGE (Op Effect Stop)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x03,	// LOGICAL_MAXIMUM (03)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x00,	// OUTPUT (Data,Ary,Abs)
  0xC0,	// END COLLECTION ()
  0x09, 0x7C,	// USAGE (Loop Count)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x46, 0xFF, 0x00,	// PHYSICAL_MAXIMUM (255)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  0x09, 0x90,	// USAGE (PID Block Free Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x0B,	// REPORT_ID (11)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01, // LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  0x09, 0x96,	// USAGE (PID Device Control)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x0C,	// REPORT_ID (12)
  0x09, 0x97,	// USAGE (DC Enable Actuators)
  0x09, 0x98,	// USAGE (DC Disable Actuators)
  0x09, 0x99,	// USAGE (DC Stop All Effects)
  0x09, 0x9A,	// USAGE (DC Device Reset)
  0x09, 0x9B,	// USAGE (DC Device Pause)
  0x09, 0x9C,	// USAGE (DC Device Continue)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x06,	// LOGICAL_MAXIMUM (06)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x00,	// OUTPUT (Data)
  0xC0,	// END COLLECTION ()

  0x09, 0x7D,	// USAGE (Device Gain Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x0D,	// REPORT_ID (14)
  0x09, 0x7E,	// USAGE (Device Gain)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x26, 0xFF, 0x00,  // LOGICAL_MAXIMUM (255)
  //0x26, 0xFF, 0x7F, // LOGICAL_MAXIMUM (32767) //milos, back to 8bit
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  //0x46, 0xFF, 0x7F, // PHYSICAL_MAXIMUM (32767) //milos
  0x75, 0x08,	// REPORT_SIZE (08)
  //0x75, 0x10,  // REPORT_SIZE (16) //milos, back to 8bit
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  //milos, commented since it was not used
  /*0x09, 0x6B,	// USAGE (Set Custom Force Report)
    0xA1, 0x02,	// COLLECTION (Logical)
    0x85, 0x0E,	// REPORT_ID (14)
    0x09, 0x22,	// USAGE (Effect Block Index)
    0x15, 0x01,	// LOGICAL_MINIMUM (01)
    0x25, 0x28,	// LOGICAL_MAXIMUM (28)
    0x35, 0x01,	// PHYSICAL_MINIMUM (01)
    0x45, 0x28,	// PHYSICAL_MAXIMUM (28)
    0x75, 0x08,	// REPORT_SIZE (08)
    0x95, 0x01,	// REPORT_COUNT (01)
    0x91, 0x02,	// OUTPUT (Data,Var,Abs)
    0x09, 0x6D,	// USAGE (Sample Count)
    0x15, 0x00,	// LOGICAL_MINIMUM (00)
    0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
    0x35, 0x00,	// PHYSICAL_MINIMUM (00)
    0x46, 0xFF, 0x00,	// PHYSICAL_MAXIMUM (255)
    0x75, 0x08,	// REPORT_SIZE (08)
    0x95, 0x01,	// REPORT_COUNT (01)
    0x91, 0x02,	// OUTPUT (Data,Var,Abs)
    0x09, 0x51,	// USAGE (Sample Period)
    0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
    0x55, 0xFD,  // UNIT_EXPONENT (-3)
    0x15, 0x00,	// LOGICAL_MINIMUM (00)
    0x26, 0xFF, 0x7F,	// LOGICAL_MAXIMUM (32767)
    0x35, 0x00,	// PHYSICAL_MINIMUM (00)
    0x46, 0xFF, 0x7F,	// PHYSICAL_MAXIMUM (32767)
    0x75, 0x10,	// REPORT_SIZE (16)
    0x95, 0x01,	// REPORT_COUNT (01)
    0x91, 0x02,	// OUTPUT (Data,Var,Abs)
    0x55, 0x00,	// UNIT_EXPONENT (00)
    0x66, 0x00, 0x00,	// UNIT (None)
    0xC0,	// END COLLECTION ()*/

  0x09, 0xAB,	// USAGE (Create New Effect Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x05,	// REPORT_ID (05)
  0x09, 0x25,	// USAGE (Effect Type)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x09, 0x26, // USAGE (ET Constant Force)
  0x09, 0x27, // USAGE (ET Ramp)
  0x09, 0x30, // USAGE (ET Square)
  0x09, 0x31, // USAGE (ET Sine)
  0x09, 0x32, // USAGE (ET Triangle)
  0x09, 0x33, // USAGE (ET Sawtooth Up)
  0x09, 0x34, // USAGE (ET Sawtooth Down)
  0x09, 0x40, // USAGE (ET Spring)
  0x09, 0x41, // USAGE (ET Damper)
  0x09, 0x42, // USAGE (ET Inertia)
  0x09, 0x43, // USAGE (ET Friction)
  //0x09, 0x28,	// USAGE (ET Custom Force data) //milos, removed custom force effect
  0x15, 0x01,  // LOGICAL_MINIMUM (01)
  0x25, 0x0B,	// LOGICAL_MAXIMUM (11) //milos, was 0x0C (12)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x0B,	// PHYSICAL_MAXIMUM (11) //milos, was 0x0C (12)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x00,	// FEATURE (Data)
  0xC0,	// END COLLECTION ()
  0x05, 0x01,	// USAGE_PAGE (Generic Desktop)
  0x09, 0x3B,	// USAGE (Byte Count)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x26, 0xFF, 0x01,	// LOGICAL_MAXIMUM (511)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x46, 0xFF, 0x01,	// PHYSICAL_MAXIMUM (511)
  0x75, 0x0A,	// REPORT_SIZE (10)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x02,	// FEATURE (Data,Var,Abs)
  0x75, 0x06,	// REPORT_SIZE (06)
  0xB1, 0x01,	// FEATURE (Constant,Ary,Abs)
  0xC0,	// END COLLECTION ()

  0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
  0x09, 0x89,	// USAGE (PID Block Load Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x06,	// REPORT_ID (06)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x02,	// FEATURE (Data,Var,Abs)
  0x09, 0x8B,	// USAGE (Block Load Status)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x09, 0x8C,	// USAGE (Block Load Success)
  0x09, 0x8D,	// USAGE (Block Load Full)
  0x09, 0x8E,	// USAGE (Block Load Error)
  0x25, 0x03,	// LOGICAL_MAXIMUM (03)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x03,	// PHYSICAL_MAXIMUM (03)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x00,	// FEATURE (Data)
  0xC0,	// END COLLECTION ()
  0x09, 0xAC,	// USAGE (RAM Pool Available)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x27, 0xFF, 0xFF, 0x00, 0x00,	// LOGICAL_MAXIMUM (65535)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x47, 0xFF, 0xFF, 0x00, 0x00,	// PHYSICAL_MAXIMUM (65535)
  0x75, 0x10,	// REPORT_SIZE (16)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x00,	// FEATURE (Data)
  0xC0,	// END COLLECTION ()

  0x09, 0x7F,	// USAGE (PID Pool Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x07,	// REPORT_ID (07)
  0x09, 0x80,	// USAGE (RAM Pool Size)
  0x75, 0x10,	// REPORT_SIZE (16)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x27, 0xFF, 0xFF, 0x00, 0x00,  // LOGICAL_MAXIMUM (65535)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x47, 0xFF, 0xFF, 0x00, 0x00,	// PHYSICAL_MAXIMUM (65535)
  0xB1, 0x02,	// FEATURE (Data,Var,Abs)
  0x09, 0x83,	// USAGE (Simultaneous Effects Max)
  0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
  0x46, 0xFF, 0x00,	// PHYSICAL_MAXIMUM (255)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x02,	// FEATURE (Data,Var,Abs)
  0x09, 0xA9,	// USAGE (Device Managed Pool)
  0x09, 0xAA,	// USAGE (Shared Parameter Blocks)
  0x75, 0x01,	// REPORT_SIZE (01)
  0x95, 0x02,	// REPORT_COUNT (02)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x25, 0x01,	// LOGICAL_MAXIMUM (01)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x45, 0x01,	// PHYSICAL_MAXIMUM (01)
  0xB1, 0x02,	// FEATURE (Data,Var,Abs)
  0x75, 0x06,	// REPORT_SIZE (06)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x03,	// FEATURE ( Cnst,Var,Abs)
  0xC0,	// END COLLECTION ()

  0xC0,	// END COLLECTION ()
#endif
};

extern const HIDDescriptor _hidInterface PROGMEM;
const HIDDescriptor _hidInterface =
{
  D_INTERFACE(HID_INTERFACE, HID_ENPOINT_COUNT, 3, 0, 0), //0xEF,0x02,0x01),
  D_HIDREPORT(sizeof(_hidReportDescriptor)),
  D_ENDPOINT(USB_ENDPOINT_IN(HID_ENDPOINT_INT), USB_ENDPOINT_TYPE_INTERRUPT, 0x40, 0x01),
#if HID_ENPOINT_COUNT>1
  D_ENDPOINT(USB_ENDPOINT_OUT(HID_ENDPOINT_OUT), USB_ENDPOINT_TYPE_INTERRUPT, 0x40, 0x01)
#endif
};

//================================================================================
//================================================================================
//	Driver

u8 _hid_protocol = 1;
u8 _hid_idle = 1;

#define WEAK __attribute__ ((weak))

int WEAK HID_GetInterface(u8* interfaceNum)
{
  interfaceNum[0] += 1;	// uses 1
  return USB_SendControl(TRANSFER_PGM, &_hidInterface, sizeof(_hidInterface));
}

int WEAK HID_GetDescriptor(int i)
{
  return USB_SendControl(TRANSFER_PGM, _hidReportDescriptor, sizeof(_hidReportDescriptor));
}

void WEAK HID_SendReport(u8 id, const void* data, int len)
{
  USB_Send(HID_TX, &id, 1);
  USB_Send(HID_TX | TRANSFER_RELEASE, data, len);
}

u8 WEAK HID_ReportAvailable()
{
  return (USB_Available(HID_RX));
}

s16 WEAK HID_ReceiveReport(void* data, int len)
{
  return (USB_Recv(HID_RX, data, len));
}

b8 WEAK HID_Setup(Setup& setup)
{
  u8 r = setup.bRequest;
  u8 requestType = setup.bmRequestType;
  if (REQUEST_DEVICETOHOST_CLASS_INTERFACE == requestType)
  {
    if (HID_GET_REPORT == r)
    {
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
        _hid_protocol = setup.wValueL;
        return true;

      case HID_SET_IDLE:
        _hid_idle = setup.wValueL;
        return true;

      case HID_SET_REPORT:
        return true;
    }
  }
  return false;
}

//================================================================================
//================================================================================
// Joystick
//  Usage: Joystick.move(x, y, throttle, buttons)
//  x & y forward/left = 0, centre = 127, back/right = 255
//  throttle max = 0, min = 255
//  8 buttons packed into 1 byte

Joystick_::Joystick_()
{
}

// 8 bits axis + 8 buttons
void Joystick_::send_8 (int8_t x, uint8_t y, uint8_t z, uint8_t buttons)
{
  u8 j[4];
  j[0] = x;
  j[1] = y;
  j[2] = z;
  j[3] = buttons;
  //HID_SendReport(Report number, array of values in same order as HID descriptor, length)
  HID_SendReport(4, j, 4);
}

// 10 bits axis + 2 buttons
void Joystick_::send_10 (int16_t x, uint16_t y, uint16_t z, uint8_t buttons)
{
  u8 j[4];
  j[0] = x;
  j[1] = ((x >> 8) & 0x3) | ((y & 0x3f) << 2);
  j[2] = ((y >> 6) & 0xf) | ((z & 0xf) << 4);
  j[3] = ((z >> 4) & 0x3f) | ((buttons & 0x3) << 6);
  //HID_SendReport(Report number, array of values in same order as HID descriptor, length)
  HID_SendReport(4, j, 4);
}

// 12 bits version + 4 buttons
void Joystick_::send_12(int16_t x, uint16_t y, uint16_t z, uint8_t buttons)
{
  u8 j[5];
  j[0] = x;
  j[1] = ((x >> 8) & 0xf) | ((y & 0xf) << 4);
  j[2] = (y >> 4) & 0xff;
  j[3] = z;
  j[4] = ((z >> 8) & 0xf) | ((buttons & 0xf) << 4);

  HID_SendReport(4, j, 5);
}

// 16+12+12 bits version + 8 buttons
void Joystick_::send_16_12_12(int16_t x, uint16_t y, uint16_t z, uint8_t buttons)
{
  u8 j[6];
  j[0] = x;
  j[1] = x >> 8;
  j[2] = y;
  j[3] = (y >> 8) & 0xf | ((z & 0xf) << 4);
  j[4] = z >> 4;
  j[5] = buttons;

  HID_SendReport(4, j, 6);
}

// 16+16+12 bits version + 12 buttons
void Joystick_::send_16_16_12(int16_t x, uint16_t y, uint16_t z, uint16_t buttons)
{
  u8 j[7];
  j[0] = x;
  j[1] = x >> 8;
  j[2] = y;
  j[3] = y >> 8;
  j[4] = z;
  j[5] = (z >> 8) & 0xf | ((buttons & 0xf) << 4);
  j[6] = buttons >> 4;

  HID_SendReport(4, j, 7);
}

// 16+12+12+12 bits version + 12 buttons
void Joystick_::send_16_12_12_12(int16_t x, uint16_t y, uint16_t z, uint16_t rx, uint32_t buttons)
{
  u8 j[8];
  j[0] = x;						//8B
  j[1] = x >> 8;					//8B
  j[2] = y;						//8B
  j[3] = (y >> 8) & 0xf | ((z & 0xf) << 4);
  j[4] = (z >> 4);					//8B
  j[5] = rx;						//8B
  j[6] = (rx >> 8) & 0xf | ((buttons & 0xf) << 4);
  j[7] = buttons >> 8 | 0x00;			//

  HID_SendReport(4, j, 8);
}
// milos, 16+10+10+10 bits version + 18 buttons
void Joystick_::send_16_10_18(int16_t x, uint16_t y, uint16_t z, uint16_t rx, uint32_t buttons)
{
  // milos, total of 8 bytes, 2B for x, 4B for y, z, rx, 2B for buttons
  u8 j[8];
  j[0] = x;
  j[1] = x >> 8;
  j[2] = rx;
  j[3] = ((rx >> 8) & 0x3) | ((z & 0x3f) << 2);
  j[4] = ((z >> 6) & 0xf) | ((y & 0xf) << 4);
  j[5] = ((y >> 4) & 0x3f) | ((buttons & 0x3) << 16);
  j[6] = buttons >> 2;
  j[7] = buttons >> 10;

  HID_SendReport(4, j, 8);
}

// milos, 16+16+10+10 bits version + 12 buttons
void Joystick_::send_16_16_10_10_12(int16_t x, uint16_t y, uint16_t z, uint16_t rx, uint32_t buttons)
{
  // milos, total of 8 bytes, 2B for x, 4.5B for y, z, rx, 1.5B for buttons
  u8 j[8];
  j[0] = x;
  j[1] = x >> 8;
  //j[2] = x >> 16; //for >16bit axis
  j[2] = y;
  j[3] = y >> 8;
  j[4] = z;
  j[5] = ((z >> 8) & 0x3) | ((rx & 0x3f) << 2);
  j[6] = ((rx >> 6) & 0xf) | ((buttons & 0xf) << 4);
  j[7] = buttons >> 4;

  HID_SendReport(4, j, 8);
}

// milos ver3, 16+16+12+12 bits version + 32 buttons
void Joystick_::send_16_16_12_12_32(int16_t x, uint16_t y, uint16_t z, uint16_t rx, uint32_t buttons)
{
  // milos, total of 11 bytes, 2B for x, 2B for y, 3B for z, rx, 4B for buttons
  u8 j[11];
  j[0] = x;
  j[1] = x >> 8;
  j[2] = y;
  j[3] = y >> 8;
  j[4] = z;
  j[5] = (z >> 8) & 0xf | ((rx & 0xf) << 4);
  j[6] = rx >> 4;
  j[7] = buttons;
  j[8] = buttons >> 8;
  j[9] = buttons >> 16;
  j[10] = buttons >> 24;

  HID_SendReport(4, j, 11);
}

// milos ver4, 16+16+12+12+12 bits version + 28 buttons
void Joystick_::send_16_16_12_12_12_28(int16_t x, uint16_t y, uint16_t z, uint16_t rx, uint16_t ry, uint32_t buttons)
{
  // milos, total of 12 bytes, 2B for x, 2B for y, 3B for z and rx, 5B for ry and buttons
  u8 j[12];
  j[0] = x;
  j[1] = x >> 8;
  j[2] = y;
  j[3] = y >> 8;
  j[4] = z;
  j[5] = (z >> 8) & 0xf | ((rx & 0xf) << 4);
  j[6] = rx >> 4;
  j[7] = ry;
  j[8] = (ry >> 8) & 0xf | ((buttons & 0xf) << 4);
  j[9] = buttons >> 4;
  j[10] = buttons >> 12;
  j[11] = buttons >> 20;

  HID_SendReport(4, j, 12);
}

// milos ver5, 16+16+12+12+12+12 bits version + 32 buttons
void Joystick_::send_16_16_12_12_12_12_32(int16_t x, uint16_t y, uint16_t z, uint16_t rx, uint16_t ry, uint16_t rz, uint32_t buttons)
{
  // milos, total of 14 bytes, 2B for x, 2B for y, 3B for z and rx, 3B for ry and rz, 4B for buttons
  u8 j[14];
  j[0] = x;
  j[1] = x >> 8;
  j[2] = y;
  j[3] = y >> 8;
  j[4] = z;
  j[5] = (z >> 8) & 0xf | ((rx & 0xf) << 4);
  j[6] = rx >> 4;
  j[7] = ry;
  j[8] = (ry >> 8) & 0xf | ((rz & 0xf) << 4);
  j[9] = rz >> 4;
  j[10] = buttons;
  j[11] = buttons >> 8;
  j[12] = buttons >> 16;
  j[13] = buttons >> 24;

  HID_SendReport(4, j, 14);
}

// DEBUG use 2 axis to H-SHIFTER
void Joystick_::send_16_8_32(int16_t x, uint16_t y, uint16_t z, uint16_t rx, uint16_t sx, uint16_t sy, uint32_t buttons)
{
  u8 j[11];
  j[0] = x;											//8B
  j[1] = x >> 8;										//8B
  j[2] = y;											//8B
  j[3] = z;		// falta 2/10   vai 6/10
  j[4] = rx;		// falta 4/10   vai 4/10
  j[5] = sx;
  j[6] = sy;
  j[7] = buttons;	// falta 6/10	vai 2/18
  j[8] = (buttons >> 8);								// 10/18
  j[9] = (buttons >> 16);								// 18/18
  j[10] = (buttons >> 24);

  HID_SendReport(4, j, 11);

}
/*
  void Joystick_::send_16_10_18(int16_t x, uint16_t y, uint16_t z, uint16_t rx, uint32_t buttons)
  {
	u8 j[8];
	j[0] = x;											//8B
	j[1] = x >> 8;										//8B
	j[2] = y;											//8B
	j[3] = ((y >> 8) & 0x3) | ((z & 0x3f) << 2);		// falta 2/10   vai 6/10
	j[4] = ((z >> 6) & 0xf) | ((rx & 0xf) << 4);		// falta 4/10   vai 4/10
	j[5] = ((rx >> 4) & 0x3f) | ((buttons & 0x3) << 6);	// falta 6/10	vai 2/18
	j[6] = (buttons >> 2);								// 10/18
	j[7] = (buttons >> 10);								// 18/18

	HID_SendReport(4, j, 8);

  }*/

// 16+16+16 bits version + 8 buttons
void Joystick_::send_16(int16_t x, uint16_t y, uint16_t z, uint8_t buttons)
{
  u8 j[7];
  j[0] = x & 0xff;
  j[1] = x >> 8;
  j[2] = y & 0xff;
  j[3] = y >> 8;
  j[4] = z & 0xff;
  j[5] = z >> 8;
  j[6] = buttons;

  HID_SendReport(4, j, 7);
}

//================================================================================
//================================================================================
//	Mouse

Mouse_::Mouse_(void) : _buttons(0)
{
}

void Mouse_::begin(void)
{
}

void Mouse_::end(void)
{
}

void Mouse_::click(uint8_t b)
{
  _buttons = b;
  move(0, 0, 0);
  _buttons = 0;
  move(0, 0, 0);
}

void Mouse_::move(signed char x, signed char y, signed char wheel)
{
  u8 m[4];
  m[0] = _buttons;
  m[1] = x;
  m[2] = y;
  m[3] = wheel;
  HID_SendReport(4, m, 4);
}

void Mouse_::buttons(uint8_t b)
{
  if (b != _buttons)
  {
    _buttons = b;
    move(0, 0, 0);
  }
}

void Mouse_::press(uint8_t b)
{
  buttons(_buttons | b);
}

void Mouse_::release(uint8_t b)
{
  buttons(_buttons & ~b);
}

bool Mouse_::isPressed(uint8_t b)
{
  if ((b & _buttons) > 0)
    return true;
  return false;
}

//================================================================================
//================================================================================
//	Keyboard

Keyboard_::Keyboard_(void)
{
}

void Keyboard_::begin(void)
{
}

void Keyboard_::end(void)
{
}

void Keyboard_::sendReport(KeyReport* keys)
{
  HID_SendReport(2, keys, sizeof(KeyReport));
}

extern
const uint8_t _asciimap[128] PROGMEM;

#define SHIFT 0x80
const uint8_t _asciimap[128] =
{
  0x00,             // NUL
  0x00,             // SOH
  0x00,             // STX
  0x00,             // ETX
  0x00,             // EOT
  0x00,             // ENQ
  0x00,             // ACK
  0x00,             // BEL
  0x2a,			// BS	Backspace
  0x2b,			// TAB	Tab
  0x28,			// LF	Enter
  0x00,             // VT
  0x00,             // FF
  0x00,             // CR
  0x00,             // SO
  0x00,             // SI
  0x00,             // DEL
  0x00,             // DC1
  0x00,             // DC2
  0x00,             // DC3
  0x00,             // DC4
  0x00,             // NAK
  0x00,             // SYN
  0x00,             // ETB
  0x00,             // CAN
  0x00,             // EM
  0x00,             // SUB
  0x00,             // ESC
  0x00,             // FS
  0x00,             // GS
  0x00,             // RS
  0x00,             // US

  0x2c,		   //  ' '
  0x1e | SHIFT,	 // !
  0x34 | SHIFT,	 // "
  0x20 | SHIFT,  // #
  0x21 | SHIFT,  // $
  0x22 | SHIFT,  // %
  0x24 | SHIFT,  // &
  0x34,          // '
  0x26 | SHIFT,  // (
  0x27 | SHIFT,  // )
  0x25 | SHIFT,  // *
  0x2e | SHIFT,  // +
  0x36,          // ,
  0x2d,          // -
  0x37,          // .
  0x38,          // /
  0x27,          // 0
  0x1e,          // 1
  0x1f,          // 2
  0x20,          // 3
  0x21,          // 4
  0x22,          // 5
  0x23,          // 6
  0x24,          // 7
  0x25,          // 8
  0x26,          // 9
  0x33 | SHIFT,    // :
  0x33,          // ;
  0x36 | SHIFT,    // <
  0x2e,          // =
  0x37 | SHIFT,    // >
  0x38 | SHIFT,    // ?
  0x1f | SHIFT,    // @
  0x04 | SHIFT,    // A
  0x05 | SHIFT,    // B
  0x06 | SHIFT,    // C
  0x07 | SHIFT,    // D
  0x08 | SHIFT,    // E
  0x09 | SHIFT,    // F
  0x0a | SHIFT,    // G
  0x0b | SHIFT,    // H
  0x0c | SHIFT,    // I
  0x0d | SHIFT,    // J
  0x0e | SHIFT,    // K
  0x0f | SHIFT,    // L
  0x10 | SHIFT,    // M
  0x11 | SHIFT,    // N
  0x12 | SHIFT,    // O
  0x13 | SHIFT,    // P
  0x14 | SHIFT,    // Q
  0x15 | SHIFT,    // R
  0x16 | SHIFT,    // S
  0x17 | SHIFT,    // T
  0x18 | SHIFT,    // U
  0x19 | SHIFT,    // V
  0x1a | SHIFT,    // W
  0x1b | SHIFT,    // X
  0x1c | SHIFT,    // Y
  0x1d | SHIFT,    // Z
  0x2f,          // [
  0x31,          // bslash
  0x30,          // ]
  0x23 | SHIFT,  // ^
  0x2d | SHIFT,  // _
  0x35,          // `
  0x04,          // a
  0x05,          // b
  0x06,          // c
  0x07,          // d
  0x08,          // e
  0x09,          // f
  0x0a,          // g
  0x0b,          // h
  0x0c,          // i
  0x0d,          // j
  0x0e,          // k
  0x0f,          // l
  0x10,          // m
  0x11,          // n
  0x12,          // o
  0x13,          // p
  0x14,          // q
  0x15,          // r
  0x16,          // s
  0x17,          // t
  0x18,          // u
  0x19,          // v
  0x1a,          // w
  0x1b,          // x
  0x1c,          // y
  0x1d,          // z
  0x2f | SHIFT,  //
  0x31 | SHIFT,  // |
  0x30 | SHIFT,  // }
  0x35 | SHIFT,  // ~
  0				// DEL
};

uint8_t USBPutChar(uint8_t c);

// press() adds the specified key (printing, non-printing, or modifier)
// to the persistent key report and sends the report.  Because of the way
// USB HID works, the host acts like the key remains pressed until we
// call release(), releaseAll(), or otherwise clear the report and resend.
size_t Keyboard_::press(uint8_t k)
{
  uint8_t i;
  if (k >= 136) {			// it's a non-printing key (not a modifier)
    k = k - 136;
  } else if (k >= 128) {	// it's a modifier key
    _keyReport.modifiers |= (1 << (k - 128));
    k = 0;
  } else {				// it's a printing key
    k = pgm_read_byte(_asciimap + k);
    if (!k) {
      setWriteError();
      return 0;
    }
    if (k & 0x80) {						// it's a capital letter or other character reached with shift
      _keyReport.modifiers |= 0x02;	// the left shift modifier
      k &= 0x7F;
    }
  }

  // Add k to the key report only if it's not already present
  // and if there is an empty slot.
  if (_keyReport.keys[0] != k && _keyReport.keys[1] != k &&
      _keyReport.keys[2] != k && _keyReport.keys[3] != k &&
      _keyReport.keys[4] != k && _keyReport.keys[5] != k) {

    for (i = 0; i < 6; i++) {
      if (_keyReport.keys[i] == 0x00) {
        _keyReport.keys[i] = k;
        break;
      }
    }
    if (i == 6) {
      setWriteError();
      return 0;
    }
  }
  sendReport(&_keyReport);
  return 1;
}

// release() takes the specified key out of the persistent key report and
// sends the report.  This tells the OS the key is no longer pressed and that
// it shouldn't be repeated any more.
size_t Keyboard_::release(uint8_t k)
{
  uint8_t i;
  if (k >= 136) {			// it's a non-printing key (not a modifier)
    k = k - 136;
  } else if (k >= 128) {	// it's a modifier key
    _keyReport.modifiers &= ~(1 << (k - 128));
    k = 0;
  } else {				// it's a printing key
    k = pgm_read_byte(_asciimap + k);
    if (!k) {
      return 0;
    }
    if (k & 0x80) {							// it's a capital letter or other character reached with shift
      _keyReport.modifiers &= ~(0x02);	// the left shift modifier
      k &= 0x7F;
    }
  }

  // Test the key report to see if k is present.  Clear it if it exists.
  // Check all positions in case the key is present more than once (which it shouldn't be)
  for (i = 0; i < 6; i++) {
    if (0 != k && _keyReport.keys[i] == k) {
      _keyReport.keys[i] = 0x00;
    }
  }

  sendReport(&_keyReport);
  return 1;
}

void Keyboard_::releaseAll(void)
{
  _keyReport.keys[0] = 0;
  _keyReport.keys[1] = 0;
  _keyReport.keys[2] = 0;
  _keyReport.keys[3] = 0;
  _keyReport.keys[4] = 0;
  _keyReport.keys[5] = 0;
  _keyReport.modifiers = 0;
  sendReport(&_keyReport);
}

size_t Keyboard_::write(uint8_t c)
{
  uint8_t p = press(c);		// Keydown
  uint8_t r = release(c);		// Keyup
  return (p);					// just return the result of press() since release() almost always returns 1
}

#endif

#endif /* if defined(USBCON) */
