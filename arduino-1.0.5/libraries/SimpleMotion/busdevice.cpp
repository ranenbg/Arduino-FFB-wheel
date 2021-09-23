#include "busdevice.h"

#include "rs232.h"
//#include <string.h>

#define BD_NONE 0
#define BD_RS 1
#define BD_FTDI 2

//how much bytes available in transmit buffer
#define TANSMIT_BUFFER_LENGTH 128

typedef struct _SMBusDevice
{
	//common
    smint8 bdType;//bus device type (such as rs232 or ftdi lib or mcu UART etc). 1=rs232 lib
	smbool opened;

    SM_STATUS cumulativeSmStatus;

	//used for rs232 lib only
	int comPort;

    smuint8 txBuffer[TANSMIT_BUFFER_LENGTH];
    smint32 txBufferUsed;//how many bytes in buffer currently

	//used for FTDI lib only
} SMBusDevice;

//init on first open
smbool bdInitialized=smfalse;
SMBusDevice BusDevice[SM_MAX_BUSES];

//init device struct table
void smBDinit()
{
	int i;
	for(i=0;i<SM_MAX_BUSES;i++)
	{
		BusDevice[i].bdType=BD_NONE;
		BusDevice[i].opened=smfalse;
        BusDevice[i].txBufferUsed=0;
	}
	bdInitialized=smtrue;
}

//ie "COM1" "VSD2USB"
//return -1 if fails, otherwise handle number
smbusdevicehandle smBDOpen( const char *devicename )
{
	int handle;

	//true on first call
	if(bdInitialized==smfalse)
		smBDinit();

	//find free handle
	for(handle=0;handle<SM_MAX_BUSES;handle++)
	{
		if(BusDevice[handle].opened==smfalse) break;//choose this
	}

	//all handles in use
	if(handle>=SM_MAX_BUSES) return -1;

//     if(strncmp(devicename,"COM",3) == 0 || strncmp(devicename,"/dev/tty",8) == 0) //use rs232 lib
 	{
            BusDevice[handle].comPort=OpenComport( devicename, SM_BAUDRATE );
                if( BusDevice[handle].comPort == -1 )
		{
			return -1; //failed to open
		}
		BusDevice[handle].bdType=BD_RS;
        BusDevice[handle].txBufferUsed=0;
	}
// 	else//no other bus types supproted yet
// 	{
// 		return -1;
// 	}

	//success
    BusDevice[handle].cumulativeSmStatus=0;
	BusDevice[handle].opened=smtrue;
	return handle;
}

smbool smIsBDHandleOpen( const smbusdevicehandle handle )
{
	if(handle<0) return smfalse;
	if(handle>=SM_MAX_BUSES) return smfalse;
	return BusDevice[handle].opened;
}

//return true if ok
smbool smBDClose( const smbusdevicehandle handle )
{
	//check if handle valid & open
	if( smIsBDHandleOpen(handle)==smfalse ) return smfalse;

	if( BusDevice[handle].bdType==BD_RS )
	{
		CloseComport( BusDevice[handle].comPort );
		BusDevice[handle].opened=smfalse;
		return smtrue;
	}

	return smfalse;
}




//write one byte to buffer and send later with smBDTransmit()
//returns true on success
smbool smBDWrite(const smbusdevicehandle handle, const smuint8 byte )
{
	//check if handle valid & open
	if( smIsBDHandleOpen(handle)==smfalse ) return smfalse;

	if( BusDevice[handle].bdType==BD_RS )
	{
        if(BusDevice[handle].txBufferUsed<TANSMIT_BUFFER_LENGTH)
        {
            //append to buffer
            BusDevice[handle].txBuffer[BusDevice[handle].txBufferUsed]=byte;
            BusDevice[handle].txBufferUsed++;
			return smtrue;
        }
		else
		{
			Serial.print(" transmit buffer too small : "); Serial.println(BusDevice[handle].txBufferUsed);
			return smfalse;
		}
	}

	return smfalse;
}

smbool smBDTransmit(const smbusdevicehandle handle)
{
    //check if handle valid & open
    if( smIsBDHandleOpen(handle)==smfalse ) return smfalse;

// 	Serial.println("smBDTransmit");
	if (BusDevice[handle].bdType == BD_RS)
    {
        if(SendBuf(BusDevice[handle].comPort,BusDevice[handle].txBuffer, BusDevice[handle].txBufferUsed)==BusDevice[handle].txBufferUsed)
        {
//			Serial.print("smTransmit : "); Serial.println(BusDevice[handle].txBufferUsed);
			BusDevice[handle].txBufferUsed = 0;
            return smtrue;
        }
        else
        {
            BusDevice[handle].txBufferUsed=0;
            return smfalse;
        }
    }
    return smfalse;
}

//read one byte from bus. if byte not immediately available, block return up to SM_READ_TIMEOUT millisecs to wait data
//returns true if byte read sucessfully
smbool smBDRead( const smbusdevicehandle handle, smuint8 *byte )
{
	//check if handle valid & open
	if( smIsBDHandleOpen(handle)==smfalse ) return smfalse;

	if( BusDevice[handle].bdType==BD_RS )
	{
		int n;
		n=PollComport(BusDevice[handle].comPort, byte, 1);
		if( n!=1 ) return smfalse;
		else return smtrue;
	}

	return smfalse;
}
