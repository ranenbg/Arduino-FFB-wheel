/*
  Force Feedback Joystick
  Basic debugging utilities.

  This code is for Microsoft Sidewinder Force Feedback Pro joystick
  with some room for additional extra controls.

  Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
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

#include <arduino.h>
#include "debug.h"

// Various debug target settings
const u8 DEBUG_TO_NONE = 0; // Disable sending debug data
const u8 DEBUG_TO_UART = 1; // Debug data sent to UART-out
const u8 DEBUG_TO_USB = 2; // Debug data sent to USB COM-port - enables basic level debugging
const u8 DEBUG_DETAIL = 4; // Include additional details to debug data

// Controls whether debug data contains data as hexadecimal ascii instead of as raw binary
#define DEBUG_DATA_AS_HEX

volatile u8 gDebugMode = 1;// DEBUG_TO_NONE;//1; // set this higher if debugging e.g. at startup is needed

#ifdef DEBUG_ENABLE_USB
// Internal buffer for sending debug data to USB COM-port
volatile char debug_buffer[DEBUG_BUFFER_SIZE];
volatile uint16_t debug_buffer_used = 0;
#endif

void LogSendData(u8 *data, uint16_t len)
{
#ifdef DEBUG_FFB
  if (gDebugMode == DEBUG_TO_NONE)
    return;

  uint16_t i = 0;
  for (i = 0; i < len; i++)
  {
#ifdef DEBUG_DATA_AS_HEX
    static const char *hexmap = "0123456789ABCDEF";
    LogSendByte( ' ' );
    LogSendByte( hexmap[(data[i] >> 4)]);
    LogSendByte( hexmap[(data[i] & 0x0F)]);
#else
    LogSendByte(data[i]);
#endif
  }
#endif
}


void LogText(const char *text)
{
#ifdef DEBUG_FFB
  if (gDebugMode == DEBUG_TO_NONE)
    return;

  while (1)
  {
    char c = *text++;
    if (c == 0)
      break;
    if (c == '\n')
      LogSendByte('\r');	// CR
    LogSendByte(c);
  }
#endif
}

void LogTextLf(const char *text)
{
#ifdef DEBUG_FFB
  if (gDebugMode == DEBUG_TO_NONE)
    return;

  LogText(text);
  LogSendByte('\r');	// CR
  LogSendByte('\n');	// LF
#endif
}


void LogTextP(const char *text)
{
#ifdef DEBUG_FFB
  if (gDebugMode == DEBUG_TO_NONE)
    return;

  while (1)
  {
    char c = pgm_read_byte(text++);
    if (c == 0)
      break;
    if (c == '\n')
      LogSendByte('\r');	// CR
    LogSendByte(c);
  }
#endif
}

void LogTextLfP(const char *text)
{
#ifdef DEBUG_FFB
  if (gDebugMode == DEBUG_TO_NONE)
    return;

  LogTextP(text);
  LogSendByte('\r');	// CR
  LogSendByte('\n');	// LF
#endif
}


void LogBinary(const void *data, uint16_t len)
{
#ifdef DEBUG_FFB
  if (gDebugMode == DEBUG_TO_NONE)
    return;

  u8 temp = (u8) (len & 0xFF);
  if (temp > 0)
    LogSendData((u8*) data, temp);
#endif
}

void LogBinaryLf(const void *data, uint16_t len)
{
#ifdef DEBUG_FFB
  if (gDebugMode == DEBUG_TO_NONE)
    return;

  u8 temp = (u8) (len & 0xFF);
  if (temp > 0)
    LogSendData((u8*) data, temp);
  LogSendByte('\r');	// CR
  LogSendByte('\n');	// LF
#endif
}

void LogData(const char *text, u8 reportId, const void *data, uint16_t len)
{
#ifdef DEBUG_FFB
  if (gDebugMode == DEBUG_TO_NONE)
    return;

  LogText(text);
  LogBinary(&reportId, 1);
  LogBinary(data, len);
#endif
}

void LogDataLf(const char *text, u8 reportId, const void *data, uint16_t len)
{
#ifdef DEBUG_FFB
  if (gDebugMode == DEBUG_TO_NONE)
    return;

  LogText(text);
  LogBinary(&reportId, 1);
  LogBinaryLf(data, len);
#endif
}

// Log all reports found in the given data (may have one or more)
void LogReport(const char *text, const uint16_t *reportSizeArray, u8 *data, uint16_t len)
{
#ifdef DEBUG_FFB
  if (gDebugMode == DEBUG_TO_NONE)
    return;

  LogTextP(text);

  u8 *p = data;

  while (p < data + len)
  {
    u8 replen = reportSizeArray[p[0] - 1];
    LogBinary(p, 1);
    if (replen > 1)
      LogBinary(&p[1], replen - 1);
    p += replen;
  }
  LogSendByte('\r');	// CR
  LogSendByte('\n');	// LF
#endif
}

b8 DoDebug(const u8 type)
{
  return ((gDebugMode & type) == type);
}

// -------------------------------
// Send debug data to/from buffer
// to the chosen bus (USB or MIDI)

void LogSendByte(u8 data)
{
#ifdef DEBUG_FFB
  DEBUG_SERIAL.write(data);
#endif
}


void FlushDebugBuffer(void)
{
}
