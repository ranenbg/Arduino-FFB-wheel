

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

#define CDC_ENABLED
#define HID_ENABLED


#ifdef CDC_ENABLED
#define CDC_INTERFACE_COUNT	2
#define CDC_ENPOINT_COUNT	3
#else
#define CDC_INTERFACE_COUNT	0
#define CDC_ENPOINT_COUNT	0
#endif

#ifdef HID_ENABLED
#define HID_INTERFACE_COUNT	1
#define HID_ENPOINT_COUNT	2
#else
#define HID_INTERFACE_COUNT	0
#define HID_ENPOINT_COUNT	0
#endif

#define CDC_ACM_INTERFACE	0	// CDC ACM
#define CDC_DATA_INTERFACE	1	// CDC Data
#define CDC_FIRST_ENDPOINT	1
#define CDC_ENDPOINT_ACM	(CDC_FIRST_ENDPOINT)							// CDC First
#define CDC_ENDPOINT_OUT	(CDC_FIRST_ENDPOINT+1)
#define CDC_ENDPOINT_IN		(CDC_FIRST_ENDPOINT+2)

#define HID_INTERFACE		(CDC_ACM_INTERFACE + CDC_INTERFACE_COUNT)		// HID Interface
#define HID_FIRST_ENDPOINT	(CDC_FIRST_ENDPOINT + CDC_ENPOINT_COUNT)	// 1 + 3
#define HID_ENDPOINT_INT	(HID_FIRST_ENDPOINT)
#define HID_ENDPOINT_OUT	(HID_FIRST_ENDPOINT+1)

#define INTERFACE_COUNT		(MSC_INTERFACE + MSC_INTERFACE_COUNT)

#ifdef CDC_ENABLED
#define CDC_RX CDC_ENDPOINT_OUT
#define CDC_TX CDC_ENDPOINT_IN
#endif

#ifdef HID_ENABLED
#define HID_TX HID_ENDPOINT_INT
#define HID_RX HID_ENDPOINT_OUT

#define NB_AXIS			     4
#define NB_FF_AXIS		   1  //1
#define NB_BUTTONS		  32  //16
#define X_AXIS_NB_BITS	16  //16
#define Y_AXIS_NB_BITS	16	//10
#define Z_AXIS_NB_BITS	12  //10
#define RX_AXIS_NB_BITS	12  //10

#define X_AXIS_LOG_MAX	((1L<<(X_AXIS_NB_BITS-1))-1)
#define X_AXIS_LOG_MIN	(-X_AXIS_LOG_MAX)
#define X_AXIS_PHYS_MAX	((1L<<X_AXIS_NB_BITS)-1)

#define Y_AXIS_LOG_MAX	((1L<<(Y_AXIS_NB_BITS))-1)
#define Y_AXIS_LOG_MIN	0//(-Y_AXIS_LOG_MAX)
#define Y_AXIS_PHYS_MAX	((1L<<Y_AXIS_NB_BITS)-1)

#define Z_AXIS_LOG_MAX	((1L<<(Z_AXIS_NB_BITS))-1)
#define Z_AXIS_LOG_MIN	0//(-Z_AXIS_LOG_MAX)
#define Z_AXIS_PHYS_MAX	((1L<<Z_AXIS_NB_BITS)-1)

#define RX_AXIS_LOG_MAX	((1L<<(RX_AXIS_NB_BITS))-1)
#define RX_AXIS_LOG_MIN	0//(-RX_AXIS_LOG_MAX)
#define RX_AXIS_PHYS_MAX	((1L<<RX_AXIS_NB_BITS)-1)

//#define SendInputReport(m_x,m_y,m_z,m_buttons)			Joystick.send_12(m_x,m_y,m_z,m_buttons)
//#define SendInputReport(m_x,m_y,m_z,m_buttons)			Joystick.send_16_12_12(m_x,m_y,m_z,m_buttons)
//#define SendInputReport(m_x,m_y,m_z,m_buttons)			Joystick.send_16_16_12(m_x,m_y,m_z,m_buttons)
//#define SendInputReport(m_x,m_y,m_z,m_rx,m_buttons)			Joystick.send_16_10_10_10(m_x,m_y,m_z,m_rx,m_buttons)
//#define SendInputReport(m_x,m_y,m_z,m_rx,m_buttons)			Joystick.send_16_16_12(m_x,m_y,m_z,m_buttons)
//#define SendInputReport(m_x,m_y,m_z,m_rx,m_buttons)			Joystick.send_16_10_18(m_x,m_y,m_z,m_rx,m_buttons) // milos, ver1
//#define SendInputReport(m_x,m_y,m_z,m_rx,sx,sy,m_buttons)			Joystick.send_16_8_32(m_x,m_y,m_z,m_rx,sx,sy,m_buttons)
//#define SendInputReport(m_x,m_y,m_z,m_rx,m_buttons)			Joystick.send_16_16_10_10_12(m_x,m_y,m_z,m_rx,m_buttons) // milos, ver2
#define SendInputReport(m_x,m_y,m_z,m_rx,m_buttons)      Joystick.send_16_16_12_12_32(m_x,m_y,m_z,m_rx,m_buttons) // milos, ver3


#endif

#define IMANUFACTURER	1
#define IPRODUCT		2


