/*
***************************************************************************
*
* Author: Teunis van Beelen
*
* Copyright (C) 2005, 2006, 2007, 2008, 2009 Teunis van Beelen
*
* teuniz@gmail.com
*
***************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*
***************************************************************************
*
* This version of GPL is at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*
***************************************************************************
*/

#include "rs232.h"
#include "simplemotion_private.h"

#ifdef __linux__   /* Linux */


int OpenComport(const char * comport_name, int baudrate)
{
    int error;
    int handle;
	int baudr;
        struct termios new_port_settings;

	switch(baudrate)
	{
		case      50 : baudr = B50;
		break;
		case      75 : baudr = B75;
		break;
		case     110 : baudr = B110;
		break;
		case     134 : baudr = B134;
		break;
		case     150 : baudr = B150;
		break;
		case     200 : baudr = B200;
		break;
		case     300 : baudr = B300;
		break;
		case     600 : baudr = B600;
		break;
		case    1200 : baudr = B1200;
		break;
		case    1800 : baudr = B1800;
		break;
		case    2400 : baudr = B2400;
		break;
		case    4800 : baudr = B4800;
		break;
		case    9600 : baudr = B9600;
		break;
		case   19200 : baudr = B19200;
		break;
		case   38400 : baudr = B38400;
		break;
		case   57600 : baudr = B57600;
		break;
		case  115200 : baudr = B115200;
		break;
		case  230400 : baudr = B230400;
		break;
		case  460800 : baudr = B460800;
		break;
		case  500000 : baudr = B500000;
		break;
		case  576000 : baudr = B576000;
		break;
		case  921600 : baudr = B921600;
		break;
		case 1000000 : baudr = B1000000;
		break;
		default      : printf("invalid baudrate\n");
		return(1);
		break;
	}

	//Cport[comport_number] = open(comports[comport_number], O_RDWR | O_NOCTTY | O_NDELAY);
        handle = open(comport_name, O_RDWR | O_NOCTTY );
        if(handle==-1)
	{
		perror("unable to open comport ");
                return(handle);
	}

    memset(&new_port_settings, 0, sizeof(new_port_settings));  /* clear the new struct */

	new_port_settings.c_cflag = baudr | CS8 | CLOCAL | CREAD;
	new_port_settings.c_iflag = IGNPAR;
	new_port_settings.c_oflag = 0;
	new_port_settings.c_lflag = 0;
	new_port_settings.c_cc[VMIN] = 0;      /* block untill n bytes are received */
	new_port_settings.c_cc[VTIME] = readTimeoutMs/100;     /* block untill a timer expires (n * 100 mSec.) */
        error = tcsetattr(handle, TCSANOW, &new_port_settings);
	if(error==-1)
	{
                close(handle);
                perror("unable to adjust portsettings ");
                return(-1);
	}

        return(handle);
}


int PollComport(int comport_number, unsigned char *buf, int size)
{
	int n;

	#ifndef __STRICT_ANSI__                       /* __STRICT_ANSI__ is defined when the -ansi option is used for gcc */
	if(size>SSIZE_MAX)  size = (int)SSIZE_MAX;  /* SSIZE_MAX is defined in limits.h */
		#else
		if(size>4096)  size = 4096;
		#endif

                n = read(comport_number, buf, size);

	return(n);
}


int SendByte(int comport_number, unsigned char byte)
{
	int n;

        n = write(comport_number, &byte, 1);
	if(n<0)  return(1);

	return(0);
}


int SendBuf(int comport_number, unsigned char *buf, int size)
{
        return(write(comport_number, buf, size));
}


void CloseComport(int comport_number)
{
        close(comport_number);
        //feature removed, not restorint old settings :
        //tcsetattr(comport_number, TCSANOW, old_port_settings + comport_number);
}

/*
Constant  Description
TIOCM_LE  DSR (data set ready/line enable)
TIOCM_DTR DTR (data terminal ready)
TIOCM_RTS RTS (request to send)
TIOCM_ST  Secondary TXD (transmit)
TIOCM_SR  Secondary RXD (receive)
TIOCM_CTS CTS (clear to send)
TIOCM_CAR DCD (data carrier detect)
TIOCM_CD  Synonym for TIOCM_CAR
TIOCM_RNG RNG (ring)
TIOCM_RI  Synonym for TIOCM_RNG
TIOCM_DSR DSR (data set ready)
*/

int IsCTSEnabled(int comport_number)
{
	int status;

        status = ioctl(comport_number, TIOCMGET, &status);

	if(status&TIOCM_CTS) return(1);
	else return(0);
}

#endif		// Linux

#ifdef __Arduino__

#include "Arduino.h"
#define SM_SERIAL		Serial1
#define SM_PIN_RE			4
#define SM_PIN_DE			5
#define SM_SET_WRITE_MODE()	SetRW(HIGH)
#define SM_SET_READ_MODE()	SetRW(LOW)

void SetRW	(b8 rw)				// rw = true -> writing
{
	digitalWrite(SM_PIN_RE,rw);
	digitalWrite(SM_PIN_DE,rw);
}

int OpenComport(const char *comport_name,int baudrate)
{
	SM_SERIAL.begin(baudrate);				// TODO  : parse comport_name to deal with other serial ports
	pinMode(SM_PIN_RE,OUTPUT);
	pinMode(SM_PIN_DE,OUTPUT);
	SM_SET_WRITE_MODE();
	return(1);
}

int PollComport(int comport_number,unsigned char *buf,int size)
{
	smuint32 now = millis();
	smuint32 tpoll = now;
	SM_SET_READ_MODE();
	while ((now - tpoll) < readTimeoutMs)
	{
		now = millis();
		if (SM_SERIAL.available() == size)
			return(SM_SERIAL.readBytes((char *)buf,size));
	}
	return(0);
}

int SendByte(int comport_number,unsigned char byte)
{
	digitalWrite(13,HIGH);
	SM_SET_WRITE_MODE();
	int n = SM_SERIAL.write(byte);
	digitalWrite(13,LOW);
	return(n);
}

int SendBuf(int comport_number,unsigned char *buf,int size)
{
	digitalWrite(13,HIGH);
	SM_SET_WRITE_MODE();
	int n = SM_SERIAL.write(buf,size);
//	Serial.print("sending : "); Serial.println(n);
	digitalWrite(13,LOW);
	return(n);
}

void CloseComport(int comport_number)
{
	// Do nothing, no need to close serial port on arduino
}

#endif		// Arduino

#ifdef WIN32			/* windows */

#ifdef DEBUG_COMPORT
//for finding problem with windows calls
void print_win32_system_error(char *name) {
    // Retrieve, format, and print out a message from the last error.  The
    // `name' that's passed should be in the form of a present tense noun
    // (phrase) such as "opening file".
    //
    char *ptr = NULL;
    FormatMessage(
                FORMAT_MESSAGE_ALLOCATE_BUFFER |
                FORMAT_MESSAGE_FROM_SYSTEM,
                0,
                GetLastError(),
                0,
                (char *)&ptr,
                1024,
                NULL);

    fprintf(stderr, "\nError %s: %ls\n", name, ptr);
    LocalFree(ptr);
}
#endif

int OpenComport(const char * comport_name, int baudrate)
{

    char baudr[64], portname[64];
    HANDLE handle;


    switch(baudrate)
    {
    case     110 : strcpy(baudr, "baud=110 data=8 parity=N stop=1");
        break;
    case     300 : strcpy(baudr, "baud=300 data=8 parity=N stop=1");
        break;
    case     600 : strcpy(baudr, "baud=600 data=8 parity=N stop=1");
        break;
    case    1200 : strcpy(baudr, "baud=1200 data=8 parity=N stop=1");
        break;
    case    2400 : strcpy(baudr, "baud=2400 data=8 parity=N stop=1");
        break;
    case    4800 : strcpy(baudr, "baud=4800 data=8 parity=N stop=1");
        break;
    case    9600 : strcpy(baudr, "baud=9600 data=8 parity=N stop=1");
        break;
    case   19200 : strcpy(baudr, "baud=19200 data=8 parity=N stop=1");
        break;
    case   38400 : strcpy(baudr, "baud=38400 data=8 parity=N stop=1");
        break;
    case   57600 : strcpy(baudr, "baud=57600 data=8 parity=N stop=1");
        break;
    case  115200 : strcpy(baudr, "baud=115200 data=8 parity=N stop=1");
        break;
    case  128000 : strcpy(baudr, "baud=128000 data=8 parity=N stop=1");
        break;
    case  256000 : strcpy(baudr, "baud=256000 data=8 parity=N stop=1");
        break;
    case  460800 : strcpy(baudr, "baud=460800 data=8 parity=N stop=1");
        break;
    default      : printf("invalid baudrate\n");
        return(-1);
        break;
    }

    strcpy(portname,"\\\\.\\");
    strcat(portname,comport_name);

    handle = CreateFileA(portname,
                         GENERIC_READ|GENERIC_WRITE,
                         0,                          /* no share  */
                         NULL,                       /* no security */
                         OPEN_EXISTING,
                         0,                          /* no threads */
                         NULL);                      /* no templates */

    if(handle==INVALID_HANDLE_VALUE)
    {
        printf("unable to open comport\n");
        return(-1);
    }

    DCB port_settings;
    memset(&port_settings, 0, sizeof(port_settings));  /* clear the new struct  */
    port_settings.DCBlength = sizeof(port_settings);

    if(!BuildCommDCBA(baudr, &port_settings))
    {
        printf("unable to set comport dcb settings\n");
        CloseHandle(handle);
        return(-1);
    }

    if(!SetCommState(handle, &port_settings))
    {
        printf("unable to set comport cfg settings\n");
        CloseHandle(handle);
        return(-1);
    }

    COMMTIMEOUTS Cptimeouts;

    Cptimeouts.ReadIntervalTimeout         = 0;
    Cptimeouts.ReadTotalTimeoutMultiplier  = 0;
    Cptimeouts.ReadTotalTimeoutConstant    = readTimeoutMs;
    Cptimeouts.WriteTotalTimeoutMultiplier = 50;
    Cptimeouts.WriteTotalTimeoutConstant   = 50;


    if(!SetCommTimeouts(handle, &Cptimeouts))
    {
        printf("unable to set comport time-out settings\n");
        CloseHandle(handle);
        return(-1);
    }

    return( (int)handle);
}


int PollComport(int comport_number, unsigned char *buf, int size)
{
    int n;

    if(size>4096)  size = 4096;

    /* added the void pointer cast, otherwise gcc will complain about */
    /* "warning: dereferencing type-punned pointer will break strict aliasing rules" */

    ReadFile((HANDLE)comport_number, buf, size, (LPDWORD)((void *)&n), NULL);

    return(n);
}


int SendByte(int comport_number, unsigned char byte)
{
    int n;

    WriteFile((HANDLE)comport_number, &byte, 1, (LPDWORD)((void *)&n), NULL);

    if(n<0)  return(1);

    return(0);
}



int SendBuf(int comport_number, unsigned char *buf, int size)
{
    int n;

    if(WriteFile((HANDLE)comport_number, buf, size, (LPDWORD)((void *)&n), NULL))
    {
        return(n);
    }

    return(-1);
}


void CloseComport(int comport_number)
{
    CloseHandle((HANDLE)comport_number);
}

#endif	// Windows

