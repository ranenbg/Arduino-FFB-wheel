//Copyright (c) Granite Devices Oy

/*
     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; version 2 of the License.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
*/

// #include <stdio.h>
// #include <string.h>
#if 1
#include "busdevice.h"
#include "sm485.h"
// #include <stdarg.h>

#include "simplemotion_private.h"

SM_STATUS smParseReturnData( smbus handle, smuint8 data );

#define HANDLE_STAT(stat) if(stat!=SM_OK)return (stat);
#define HANDLE_STAT_AND_RET(stat,returndata) { if(returndata==RET_INVALID_CMD||returndata==RET_INVALID_PARAM) return SM_ERR_PARAMETER; if(stat!=SM_OK) return (stat); }

enum RecvState {WaitCmdId,WaitAddr,WaitPayloadSize,WaitPayload,WaitCrcHi,WaitCrcLo};
FILE *smDebugOut=NULL;

//useful macros from extracting/storing multibyte values from/to byte buffer
#define bufput32bit(buf, pos, val) *((smuint32*)(smuint8*)((buf)+(pos)))=((smuint32)(val))
#define bufput16bit(buf, pos, val) *((smuint16*)(smuint8*)((buf)+(pos)))=((smuint16)(val))
#define bufput8bit(buf, pos, val) *((smuint8*)(smuint8*)((buf)+(pos)))=((smuint8)(val))
#define bufget32bit(buf, pos) (*((smuint32*)(smuint8*)((buf)+(pos))))
#define bufget16bit(buf, pos) (*((smuint16*)(smuint8*)((buf)+(pos))))
#define bufget8bit(buf, pos) (*((smuint8*)(smuint8*)((buf)+(pos))))


smbool smIsHandleOpen( const smbus handle );

SM_STATUS smReceiveReturnPacket( smbus bushandle );

typedef struct SM_BUS_
{
    smint8 bdHandle;
    smbool opened;

    enum RecvState recv_state,recv_state_next;

    smint16 recv_payloadsize;
    smint16 recv_storepos;
    smuint8 recv_rsbuf[SM485_RSBUFSIZE];
    smuint8 recv_cmdid;
    smuint8 recv_addr;
    smuint16 recv_crc;
    smuint16 recv_read_crc_hi;
    smbool receiveComplete;
    smbool transmitBufFull;//set true if user uploads too much commands in one SM transaction. if true, on execute commands, nothing will be sent to bus to prevent unvanted clipped commands and buffer will be cleared
    char busDeviceName[SM_BUSDEVICENAME_LEN];

    smint16 cmd_send_queue_bytes;//for queued device commands
    smint16 cmd_recv_queue_bytes;//recv_queue_bytes counted upwards at every smGetQueued.. and compared to payload size


    SM_STATUS cumulativeSmStatus;
} SM_BUS;


SM_BUS smBus[SM_MAX_BUSES];
smuint16 readTimeoutMs=SM_READ_TIMEOUT;

//init on first smOpenBus call
smbool smInitialized=smfalse;

//if debug message has priority this or above will be printed to debug stream
smVerbosityLevel smDebugThreshold=Trace;

#ifdef ENABLE_DEBUG_PRINTS
void smDebug( smbus handle, smVerbosityLevel verbositylevel, char *format, ...)
{
    va_list fmtargs;
    char buffer[1024];


    //check if bus handle is valid & opened
    if(smIsHandleOpen(handle)==smfalse) return;

    if(smDebugOut!=NULL && verbositylevel <= smDebugThreshold )
    {
        va_start(fmtargs,format);
        vsnprintf(buffer,sizeof(buffer)-1,format,fmtargs);
        va_end(fmtargs);
//         fprintf(smDebugOut,"%s: %s",smBus[handle].busDeviceName, buffer);
		Serial.println(buffer);
    }

}
#else
#define smDebug(...)
#endif

void smResetSM485variables(smbus handle)
{
    smBus[handle].recv_state=WaitCmdId;
    smBus[handle].recv_state_next=WaitCmdId;
    smBus[handle].recv_payloadsize=-1;
    smBus[handle].recv_storepos=0;//number of bytes to expect data in cmd, -1=wait cmd header
    smBus[handle].recv_cmdid=0;// cmdid=0 kun ed komento suoritettu
    smBus[handle].recv_addr=255;
    smBus[handle].recv_crc=SM485_CRCINIT;
    smBus[handle].recv_read_crc_hi=0xffff;//bottom bits will be contains only 1 byte when read
    smBus[handle].receiveComplete=smfalse;
    smBus[handle].transmitBufFull=smfalse;
    smBus[handle].cmd_send_queue_bytes=0;
    smBus[handle].cmd_recv_queue_bytes=0;
}


smuint16 calcCRC16(smuint8 data, smuint16 crc)
{
    unsigned int i; /* will index into CRC lookup */

    i = (crc>>8) ^ data; /* calculate the CRC  */
    crc = (((crc&0xff) ^ table_crc16_hi[i])<<8) | table_crc16_lo[i];

    return crc;
}

smuint16 calcCRC16Buf(const char *buffer, smuint16 buffer_length)
{
    smuint8 crc_hi = 0xFF; /* high CRC byte initialized */
    smuint8 crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
        crc_hi = crc_lo ^ table_crc16_hi[i];
        crc_lo = table_crc16_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

SM_STATUS smSetTimeout( smuint16 millsecs )
{
    if(millsecs<=5000)
    {
        readTimeoutMs=millsecs;
        return SM_OK;
    }
    return SM_ERR_PARAMETER;
}

unsigned long smGetVersion()
{
    return SM_VERSION;
}


//init bus struct table
void smBusesInit()
{
    int i;
    for(i=0;i<SM_MAX_BUSES;i++)
    {
        smBus[i].opened=smfalse;;
        smResetSM485variables(i);
    }
    smInitialized=smtrue;
}

smbool smIsHandleOpen( const smbus handle )
{
    if(handle<0) return smfalse;
    if(handle>=SM_MAX_BUSES) return smfalse;
    return smBus[handle].opened;
}


/** Open SM RS485 communication bus. Parameters:
-devicename: on Windows COM port as "COMx" or on unix /dev/ttySx or /dev/ttyUSBx where x=port number
-return value: integer handle to be used with all other commands, -1 if fails
*/
smbus smOpenBus( const char * devicename )
{
	smbus handle;

	//true on first call
	if (smInitialized == smfalse)
		smBusesInit();

    //find free handle
    for(handle=0;handle<SM_MAX_BUSES;handle++)
    {
        if(smBus[handle].opened==smfalse) break;//choose this
    }
    //all handles in use
    if(handle>=SM_MAX_BUSES) return -1;

    //open bus device
	smBus[handle].bdHandle = smBDOpen(devicename);
    if(smBus[handle].bdHandle==-1) return -1;

    //success
    strncpy( smBus[handle].busDeviceName, devicename, SM_BUSDEVICENAME_LEN );
    smBus[handle].busDeviceName[SM_BUSDEVICENAME_LEN-1]=0;//null terminate string
    smBus[handle].opened=smtrue;
    return handle;
}

/** Close connection to given bus handle number. This frees communication link therefore makes it available for other apps for opening.
-return value: a SM_STATUS value, i.e. SM_OK if command succeed
*/
LIB SM_STATUS smCloseBus( const smbus bushandle )
{
    //check if bus handle is valid & opened
    if(smIsHandleOpen(bushandle)==smfalse) return recordStatus(bushandle,SM_ERR_NODEVICE);

    smBus[bushandle].opened=smfalse;

    if( smBDClose(smBus[bushandle].bdHandle) == smfalse ) return recordStatus(bushandle,SM_ERR_BUS);

    return recordStatus(bushandle,SM_OK);
}

char *cmdidToStr(smuint8 cmdid )
{
    char *str;
    switch(cmdid)
    {
    case SMCMD_INSTANT_CMD : str="SMCMD_INSTANT_CMD";break;
    case SMCMD_INSTANT_CMD_RET :str="SMCMD_INSTANT_CMD_RET";break;
    case SMCMD_BUFFERED_CMD :str="SMCMD_BUFFERED_CMD";break;
    case SMCMD_BUFFERED_RETURN_DATA :str="SMCMD_BUFFERED_RETURN_DATA";break;
    case SMCMD_BUFFERED_RETURN_DATA_RET :str="SMCMD_BUFFERED_RETURN_DATA_RET";break;
    case SMCMD_BUFFERED_CMD_RET:str="SMCMD_BUFFERED_CMD_RET";break;
#ifdef PROCESS_IMAGE_SUPPORT
    case SMCMD_PROCESS_IMAGE:str="SMCMD_PROCESS_IMAGE";break;
    case SMCMD_PROCESS_IMAGE_RET: str="SMCMD_PROCESS_IMAGE_RET";break;
#endif
    case SMCMD_GET_CLOCK: str="SMCMD_GET_CLOCK";break;
    case SMCMD_GET_CLOCK_RET :str="SMCMD_GET_CLOCK_RET";break;
    default: str="unknown cmdid";break;
    }
    //puts(str);
    return str;
}

//write one byte to tx buffer
//returns true on success
smbool smWriteByte( const smbus handle, const smuint8 byte, smuint16 *crc )
{
	smbool success = smBDWrite(smBus[handle].bdHandle,byte);
    if(crc!=NULL)
        *crc = calcCRC16(byte,*crc);

    if(success==smtrue)
        smDebug(handle, High, "  sent byte %02x\n",byte);
    else
        smDebug(handle, High, "  sending byte %02x failed\n",byte);

    return success;
}

//write tx buffer to bus
//returns true on success
smbool smTransmitBuffer( const smbus handle )
{
    //check if bus handle is valid & opened
    if(smIsHandleOpen(handle)==smfalse) return SM_ERR_NODEVICE;

	smbool success = smBDTransmit(smBus[handle].bdHandle);
    return success;
}

SM_STATUS smSendSMCMD( smbus handle, smuint8 cmdid, smuint8 addr, smuint8 datalen, smuint8 *cmddata )
{
    int i;
    smuint8 data;
    smuint16 sendcrc;

	sendcrc = SM485_CRCINIT;

    smDebug(handle, Mid, "> %s (id=%d, addr=%d, payload=%d)\n",cmdidToStr(cmdid),cmdid,
            addr,
            datalen);


    smDebug(handle,High,"ID ");
    if( smWriteByte(handle,cmdid, &sendcrc) != smtrue ) return recordStatus(handle,SM_ERR_BUS);

	if (cmdid&SMCMD_MASK_N_PARAMS)
    {
        smDebug(handle,High,"Nparams ");
        if( smWriteByte(handle,datalen, &sendcrc) != smtrue ) return recordStatus(handle,SM_ERR_BUS);
    }

    smDebug(handle,High,"ADDR ");
    if( smWriteByte(handle,addr, &sendcrc) != smtrue ) return recordStatus(handle,SM_ERR_BUS);

    for(i=0;i<datalen;i++)
    {
        smDebug(handle,High,"DATA ");
        if( smWriteByte(handle,cmddata[i], &sendcrc) != smtrue ) return recordStatus(handle,SM_ERR_BUS);
    }

	smDebug(handle,High,"CRC ");
    if( smWriteByte(handle,sendcrc>>8, NULL)  != smtrue ) return recordStatus(handle,SM_ERR_BUS);
    smDebug(handle,High,"CRC ");
    if( smWriteByte(handle,sendcrc&0xff,NULL) != smtrue ) return recordStatus(handle,SM_ERR_BUS);

    //transmit bytes to bus that were written in buffer by smWriteByte calls
    if( smTransmitBuffer(handle) != smtrue ) return recordStatus(handle,SM_ERR_BUS);

	return recordStatus(handle,SM_OK);
}




SM_STATUS smReceiveErrorHandler( smbus handle, smbool flushrx )
{
    //check if bus handle is valid & opened
    if(smIsHandleOpen(handle)==smfalse) return SM_ERR_NODEVICE;


    //empty pending rx buffer to avoid further parse errors
    if(flushrx==smtrue)
    {
        smbool success;
        do{
            smuint8 rx;
            success=smBDRead(handle,&rx);
        }while(success==smtrue);
    }
    smResetSM485variables(handle);
    smBus[handle].receiveComplete=smtrue;
    return recordStatus(handle,SM_ERR_COMMUNICATION);
}


SM_STATUS smAppendSMCommandToQueue( smbus handle, int smpCmdType,smint32 paramvalue  )
{
    smuint8 txbyte;
    int cmdlength;

    //check if bus handle is valid & opened
    if(smIsHandleOpen(handle)==smfalse) return SM_ERR_NODEVICE;

    switch(smpCmdType)
    {
    case SMPCMD_SETPARAMADDR:
        cmdlength=2;
        break;
    case SMPCMD_24B:
        cmdlength=3;
        break;
    case SMPCMD_32B:
        cmdlength=4;
        break;
    default:
        return recordStatus(handle,SM_ERR_PARAMETER);
        break;
    }

    //check if space if buffer
    if(smBus[handle].cmd_send_queue_bytes>(SM485_MAX_PAYLOAD_BYTES-cmdlength) )
    {
        smBus[handle].transmitBufFull=smtrue; //when set true, smExecute will do nothing but clear transmit buffer. so this prevents any of overflowed commands getting thru
		return recordStatus(handle,SM_ERR_PARAMETER);// SM_ERR_LENGTH); //overflow, too many commands in buffer
    }

    if(smpCmdType==SMPCMD_SETPARAMADDR)
    {
        SMPayloadCommand16 newcmd;
        newcmd.ID=SMPCMD_SETPARAMADDR;
        newcmd.param=paramvalue;

        //          bufput8bit( smBus[handle].recv_rsbuf, smBus[handle].cmd_send_queue_bytes, 5);
        //            bufput8bit( smBus[handle].recv_rsbuf, smBus[handle].cmd_send_queue_bytes, 6);
        /*
FIXME
            ei toimi, menee vaa nollaa*/
        bufput8bit( smBus[handle].recv_rsbuf, smBus[handle].cmd_send_queue_bytes++, ((unsigned char*)&newcmd)[1]);
        bufput8bit( smBus[handle].recv_rsbuf, smBus[handle].cmd_send_queue_bytes++, ((unsigned char*)&newcmd)[0]);
    }
    if(smpCmdType==SMPCMD_24B)
    {
        SMPayloadCommand24 newcmd;
        newcmd.ID=SMPCMD_24B;
        newcmd.param=paramvalue;
        bufput8bit( smBus[handle].recv_rsbuf, smBus[handle].cmd_send_queue_bytes++, ((unsigned char*)&newcmd)[2]);
        bufput8bit( smBus[handle].recv_rsbuf, smBus[handle].cmd_send_queue_bytes++, ((unsigned char*)&newcmd)[1]);
        bufput8bit( smBus[handle].recv_rsbuf, smBus[handle].cmd_send_queue_bytes++, ((unsigned char*)&newcmd)[0]);
    }
    if(smpCmdType==SMPCMD_32B)
    {
        SMPayloadCommand32 newcmd;
        newcmd.ID=SMPCMD_32B;
        newcmd.param=paramvalue;
        bufput8bit( smBus[handle].recv_rsbuf, smBus[handle].cmd_send_queue_bytes++, ((unsigned char*)&newcmd)[3]);
        bufput8bit( smBus[handle].recv_rsbuf, smBus[handle].cmd_send_queue_bytes++, ((unsigned char*)&newcmd)[2]);
        bufput8bit( smBus[handle].recv_rsbuf, smBus[handle].cmd_send_queue_bytes++, ((unsigned char*)&newcmd)[1]);
        bufput8bit( smBus[handle].recv_rsbuf, smBus[handle].cmd_send_queue_bytes++, ((unsigned char*)&newcmd)[0]);
    }


    return recordStatus(handle,SM_OK);
}


SMPayloadCommandRet32 smConvertToPayloadRet32_16(SMPayloadCommandRet16 in)
{
    SMPayloadCommandRet32 out;
    out.ID=in.ID;
    out.retData=(long)in.retData; //negative value, extend ones
    return out;
}

//for library internal use only
SM_STATUS smTransmitReceiveCommandQueue( const smbus bushandle, const smaddr targetaddress, smuint8 cmdid )
{
    SM_STATUS stat;

	if (smBus[bushandle].transmitBufFull != smtrue) //dont send/receive commands if queue was overflowed by user error
    {
		stat = smSendSMCMD(bushandle,cmdid,targetaddress,smBus[bushandle].cmd_send_queue_bytes,smBus[bushandle].recv_rsbuf); //send commands to bus
		if (stat != SM_OK) return recordStatus(bushandle,stat);
    }

    smBus[bushandle].cmd_send_queue_bytes=0;
    smBus[bushandle].cmd_recv_queue_bytes=0;//counted upwards at every smGetQueued.. and compared to payload size

    if(smBus[bushandle].transmitBufFull!=smtrue)//dont send/receive commands if queue was overflowed by user error
    {
        stat=smReceiveReturnPacket(bushandle);//blocking wait & receive return values from bus
        if(stat!=SM_OK) return recordStatus(bushandle,stat); //maybe timeouted
    }

    smBus[bushandle].transmitBufFull=smfalse;//reset overflow status
	return recordStatus(bushandle,SM_OK);
}


SM_STATUS smExecuteCommandQueue( const smbus bushandle, const smaddr targetaddress )
{
    return recordStatus(bushandle,smTransmitReceiveCommandQueue(bushandle,targetaddress,SMCMD_INSTANT_CMD));
}

SM_STATUS smUploadCommandQueueToDeviceBuffer( const smbus bushandle, const smaddr targetaddress )
{
    return recordStatus(bushandle,smTransmitReceiveCommandQueue(bushandle,targetaddress,SMCMD_BUFFERED_CMD));
}

//return number of how many bytes waiting to be read with smGetQueuedSMCommandReturnValue
SM_STATUS smBytesReceived( const smbus bushandle, smint32 *bytesinbuffer )
{
    if(smIsHandleOpen(bushandle)==smfalse) return recordStatus(bushandle,SM_ERR_NODEVICE);

    smint32 bytes=smBus[bushandle].recv_payloadsize - smBus[bushandle].cmd_recv_queue_bytes;//how many bytes waiting to be read with smGetQueuedSMCommandReturnValue
    *bytesinbuffer=bytes;

    return recordStatus(bushandle,SM_OK);
}

SM_STATUS smGetQueuedSMCommandReturnValue(  const smbus bushandle, smint32 *retValue )
{
    smuint8 rxbyte, rettype;

    //check if bus handle is valid & opened
    if(smIsHandleOpen(bushandle)==smfalse) return SM_ERR_NODEVICE;


    //if get called so many times that receive queue buffer is already empty, return error
    if(smBus[bushandle].cmd_recv_queue_bytes>=smBus[bushandle].recv_payloadsize)
    {

		smDebug(bushandle,Trace,"Packet receive error, return data coudn't be parsed\n");

        //return 0
        if(retValue!=NULL) *retValue=0;//check every time if retValue is set NULL by caller -> don't store anything to it if its NULL

        return recordStatus(bushandle,SM_ERR_LENGTH);//not a single byte left
    }

    //get first byte to deterime packet length
    rxbyte=bufget8bit(smBus[bushandle].recv_rsbuf, smBus[bushandle].cmd_recv_queue_bytes++);
    rettype=rxbyte>>6; //extract ret packet header 2 bits

    //read rest of data based on packet header:

    if(rettype == SMPRET_16B)
    {
        //extract return packet and convert to 32 bit and return
        SMPayloadCommandRet16 read;
        smuint8 *readBuf=(smuint8*)&read;
        readBuf[1]=rxbyte;
        readBuf[0]=bufget8bit(smBus[bushandle].recv_rsbuf, smBus[bushandle].cmd_recv_queue_bytes++);
		smDebug(bushandle,Trace,"RET16B: %d\n",read.retData);

        if(retValue!=NULL) *retValue=read.retData;
        return recordStatus(bushandle,SM_OK);
    }
    if(rettype == SMPRET_24B)
    {
        //extract return packet and convert to 32 bit and return
        SMPayloadCommandRet24 read;
        smuint8 *readBuf=(smuint8*)&read;
        readBuf[2]=rxbyte;
        readBuf[1]=bufget8bit(smBus[bushandle].recv_rsbuf, smBus[bushandle].cmd_recv_queue_bytes++);
        readBuf[0]=bufget8bit(smBus[bushandle].recv_rsbuf, smBus[bushandle].cmd_recv_queue_bytes++);
		smDebug(bushandle,Trace,"RET24B: %d\n",read.retData);

        if(retValue!=NULL) *retValue=read.retData;
        return recordStatus(bushandle,SM_OK);
    }
    if(rettype == SMPRET_32B)
    {
        //extract return packet and convert to 32 bit and return
        SMPayloadCommandRet32 read;
        smuint8 *readBuf=(smuint8*)&read;
        readBuf[3]=rxbyte;
        readBuf[2]=bufget8bit(smBus[bushandle].recv_rsbuf, smBus[bushandle].cmd_recv_queue_bytes++);
        readBuf[1]=bufget8bit(smBus[bushandle].recv_rsbuf, smBus[bushandle].cmd_recv_queue_bytes++);
        readBuf[0]=bufget8bit(smBus[bushandle].recv_rsbuf, smBus[bushandle].cmd_recv_queue_bytes++);
		smDebug(bushandle,Trace,"RET32B: %d\n",read.retData);

        if(retValue!=NULL) *retValue=read.retData;
        return recordStatus(bushandle,SM_OK);
    }
    if(rettype == SMPRET_OTHER) //8bit
    {
        //extract return packet and convert to 32 bit and return
        SMPayloadCommandRet8 read;
        smuint8 *readBuf=(smuint8*)&read;
        readBuf[0]=rxbyte;
		smDebug(bushandle,Trace,"RET_OTHER: %d\n",read.retData);

        if(retValue!=NULL) *retValue=read.retData;
        return recordStatus(bushandle,SM_OK);
    }

    return recordStatus(bushandle,SM_ERR_PARAMETER); //something went wrong, rettype not known
}


SM_STATUS smReceiveReturnPacket( smbus bushandle )
{
    //check if bus handle is valid & opened
    if(smIsHandleOpen(bushandle)==smfalse) return SM_ERR_NODEVICE;

    smDebug(bushandle, High, "  Reading reply packet\n");
    do
    {
        smuint8 ret;
        SM_STATUS stat;

        smbool succ=smBDRead(bushandle,&ret);

        if(succ==smfalse)
        {
            smReceiveErrorHandler(bushandle,smfalse);
            smDebug(bushandle, High, "  smRawCommand RX read byte failed\n");
            return recordStatus(bushandle,SM_ERR_COMMUNICATION);
        }
        else
        {
            smDebug(bushandle, High, "  got byte %02x \n",ret);
        }

        stat=smParseReturnData( bushandle, ret );
        if(stat!=SM_OK) return recordStatus(bushandle,stat);
    } while(smBus[bushandle].receiveComplete==smfalse); //loop until complete packaget has been read

    //return data read complete
    smDebug(bushandle,Mid, "< %s (id=%d, addr=%d, payload=%d)\n",
            cmdidToStr( smBus[bushandle].recv_cmdid ),
            smBus[bushandle].recv_cmdid,
            smBus[bushandle].recv_addr,
            smBus[bushandle].recv_payloadsize);

    return recordStatus(bushandle,SM_OK);
}


/** Set stream where debug output is written. By default nothing is written. */
LIB void smSetDebugOutput( smVerbosityLevel level, FILE *stream )
{
    smDebugThreshold=level;
    smDebugOut=stream;
}


//short returndata16=0, payload=0;
//can be called at any frequency
SM_STATUS smParseReturnData( smbus handle, smuint8 data )
{
    //check if bus handle is valid & opened
    if(smIsHandleOpen(handle)==smfalse) return SM_ERR_NODEVICE;

    //buffered variable allows placing if's in any order (because recv_state may changes in this function)
    smBus[handle].recv_state=smBus[handle].recv_state_next;
    smBus[handle].receiveComplete=smfalse;//overwritten to true later if complete

    if(smBus[handle].recv_state==WaitPayload)
    {
        smBus[handle].recv_crc=calcCRC16(data,smBus[handle].recv_crc);

        //normal handling for all payload data
        if(smBus[handle].recv_storepos<SM485_MAX_PAYLOAD_BYTES)
            smBus[handle].recv_rsbuf[smBus[handle].recv_storepos++]=data;
        else//rx payload buffer overflow
        {
            return recordStatus(handle,(smReceiveErrorHandler(handle,smtrue)));
        }

        //all received
        if(smBus[handle].recv_payloadsize<=smBus[handle].recv_storepos)
            smBus[handle].recv_state_next=WaitCrcHi;

        return recordStatus(handle,SM_OK);
    }


    if(smBus[handle].recv_state==WaitCmdId)
    {
        smBus[handle].recv_crc=calcCRC16(data,smBus[handle].recv_crc);
        smBus[handle].recv_cmdid=data;
        switch(data&SMCMD_MASK_PARAMS_BITS)//commands with fixed payload size
        {
        case SMCMD_MASK_2_PARAMS: smBus[handle].recv_payloadsize=2; smBus[handle].recv_state_next=WaitAddr; break;
        case SMCMD_MASK_0_PARAMS: smBus[handle].recv_payloadsize=0; smBus[handle].recv_state_next=WaitAddr; break;
        case SMCMD_MASK_N_PARAMS: smBus[handle].recv_payloadsize=-1; smBus[handle].recv_state_next=WaitPayloadSize;break;//-1 = N databytes
        default:
            return recordStatus(handle,(smReceiveErrorHandler(handle, smtrue)));
            break; //error, unsupported command id
        }

        return recordStatus(handle,SM_OK);
    }

    //no data payload size known yet
    if(smBus[handle].recv_state==WaitPayloadSize)
    {
        smBus[handle].recv_crc=calcCRC16(data,smBus[handle].recv_crc);
        smBus[handle].recv_payloadsize=data;
        smBus[handle].recv_state_next=WaitAddr;
        return recordStatus(handle,SM_OK);
    }

    if(smBus[handle].recv_state==WaitAddr)
    {
        smBus[handle].recv_crc=calcCRC16(data,smBus[handle].recv_crc);
        smBus[handle].recv_addr=data;//can be receiver or sender addr depending on cmd
        if(smBus[handle].recv_payloadsize>smBus[handle].recv_storepos)
            smBus[handle].recv_state_next=WaitPayload;
        else
            smBus[handle].recv_state_next=WaitCrcHi;
        return recordStatus(handle,SM_OK);
    }

    if(smBus[handle].recv_state==WaitCrcHi)
    {
        smBus[handle].recv_read_crc_hi=data;//crc_msb
        smBus[handle].recv_state_next=WaitCrcLo;
        return recordStatus(handle,SM_OK);
    }

    //get crc_lsb, check crc and execute
    if(smBus[handle].recv_state==WaitCrcLo)
    {
        if(((smBus[handle].recv_read_crc_hi<<8)|data)!=smBus[handle].recv_crc)
        {
            //CRC error
            return recordStatus(handle,(smReceiveErrorHandler(handle,smtrue)));
        }
        else
        {
            //CRC ok
            //if(smBus[handle].recv_addr==config.deviceAddress || smBus[handle].recv_cmdid==SMCMD_GET_CLOCK_RET || smBus[handle].recv_cmdid==SMCMD_PROCESS_IMAGE ) executeSMcmd();
            smBus[handle].receiveComplete=smtrue;
        }

        //smResetSM485variables(handle);
        smBus[handle].recv_storepos=0;
        smBus[handle].recv_crc=SM485_CRCINIT;
        smBus[handle].recv_state_next=WaitCmdId;
        return recordStatus(handle,SM_OK);
    }

    return recordStatus(handle,SM_OK);
}


//reply consumes 16 bytes in payload buf, so max calls per cycle is 7
SM_STATUS smAppendGetParamCommandToQueue( smbus handle, smint16 paramAddress )
{
    SM_STATUS stat=SM_NONE;

    //check if bus handle is valid & opened
    if(smIsHandleOpen(handle)==smfalse) return SM_ERR_NODEVICE;

    //possible errors will set bits to stat
    stat|=smAppendSMCommandToQueue( handle, SMPCMD_SETPARAMADDR, SMP_RETURN_PARAM_LEN ); //2b
    stat|=smAppendSMCommandToQueue( handle, SMPCMD_24B, SMPRET_32B );//3b
    stat|=smAppendSMCommandToQueue( handle, SMPCMD_SETPARAMADDR, SMP_RETURN_PARAM_ADDR );//2b
    stat|=smAppendSMCommandToQueue( handle, SMPCMD_24B, paramAddress );//3b
    //=10 bytes

    return recordStatus(handle,stat);
}

SM_STATUS smGetQueuedGetParamReturnValue(  const smbus bushandle, smint32 *retValue )
{
    smint32 retVal=0;
    SM_STATUS stat=SM_NONE;

    //check if bus handle is valid & opened
    if(smIsHandleOpen(bushandle)==smfalse) return SM_ERR_NODEVICE;

    //must get all inserted commands from buffer
    stat|=smGetQueuedSMCommandReturnValue( bushandle, &retVal );//4x4b
    stat|=smGetQueuedSMCommandReturnValue( bushandle, &retVal );
    stat|=smGetQueuedSMCommandReturnValue( bushandle, &retVal );
    stat|=smGetQueuedSMCommandReturnValue( bushandle, &retVal );  //the real return value is here
    if(retValue!=NULL) *retValue=retVal;
    return recordStatus(bushandle,stat);
}

//consumes 6 bytes in payload buf, so max calls per cycle is 20
SM_STATUS smAppendSetParamCommandToQueue( smbus handle, smint16 paramAddress, smint32 paramValue )
{
    SM_STATUS stat=SM_NONE;

	//check if bus handle is valid & opened
    if(smIsHandleOpen(handle)==smfalse) return SM_ERR_NODEVICE;

    stat|=smAppendSMCommandToQueue( handle, SMPCMD_SETPARAMADDR, paramAddress );//2b
    stat|=smAppendSMCommandToQueue( handle, SMPCMD_32B, paramValue );//4b
    return recordStatus(handle,stat);
}


SM_STATUS smGetQueuedSetParamReturnValue(  const smbus bushandle, smint32 *retValue )
{
    smint32 retVal=0;
    SM_STATUS stat=SM_NONE;

    //check if bus handle is valid & opened
    if(smIsHandleOpen(bushandle)==smfalse) return SM_ERR_NODEVICE;

    //must get all inserted commands from buffer
    stat|=smGetQueuedSMCommandReturnValue( bushandle, &retVal );
    stat|=smGetQueuedSMCommandReturnValue( bushandle, &retVal );  //the real return value is here
    if(retValue!=NULL) *retValue=retVal;

    return recordStatus(bushandle,stat);
}


SM_STATUS smGetBufferClock( const smbus handle, const smaddr targetaddr, smuint16 *clock )
{
    SM_STATUS stat;

    //check if bus handle is valid & opened
    if(smIsHandleOpen(handle)==smfalse) return recordStatus(handle,SM_ERR_NODEVICE);

    stat=smSendSMCMD(handle, SMCMD_GET_CLOCK ,targetaddr, 0, NULL ); //send get clock commands to bus
    if(stat!=SM_OK) return recordStatus(handle,stat);

    stat=smReceiveReturnPacket(handle);//blocking wait & receive return values from bus
    if(stat!=SM_OK) return recordStatus(handle,stat); //maybe timeouted

    if(clock!=NULL)
        *clock=bufget16bit(smBus[handle].recv_rsbuf,0);

    smBus[handle].recv_storepos=0;

    return recordStatus(handle,SM_OK);
}

/** Simple read & write of parameters with internal queueing, so only one call needed.
Use these for non-time critical operations. */
SM_STATUS smRead1Parameter( const smbus handle, const smaddr nodeAddress, const smint16 paramId1, smint32 *paramVal1 )
{
    SM_STATUS smStat=0;

    smStat|=smAppendGetParamCommandToQueue(handle,paramId1);
    smStat|=smExecuteCommandQueue(handle,nodeAddress);
    smStat|=smGetQueuedGetParamReturnValue(handle,paramVal1);

    return recordStatus(handle,smStat);
}

SM_STATUS smRead2Parameters( const smbus handle, const smaddr nodeAddress, const smint16 paramId1, smint32 *paramVal1,const smint16 paramId2, smint32 *paramVal2 )
{
    SM_STATUS smStat=0;

    smStat|=smAppendGetParamCommandToQueue(handle,paramId1);
    smStat|=smAppendGetParamCommandToQueue(handle,paramId2);
    smStat|=smExecuteCommandQueue(handle,nodeAddress);
    smStat|=smGetQueuedGetParamReturnValue(handle,paramVal1);
    smStat|=smGetQueuedGetParamReturnValue(handle,paramVal2);

    return recordStatus(handle,smStat);
}

SM_STATUS smRead3Parameters( const smbus handle, const smaddr nodeAddress, const smint16 paramId1, smint32 *paramVal1,const smint16 paramId2, smint32 *paramVal2 ,const smint16 paramId3, smint32 *paramVal3 )
{
    SM_STATUS smStat=0;

    smStat|=smAppendGetParamCommandToQueue(handle,paramId1);
    smStat|=smAppendGetParamCommandToQueue(handle,paramId2);
    smStat|=smAppendGetParamCommandToQueue(handle,paramId3);
    smStat|=smExecuteCommandQueue(handle,nodeAddress);
    smStat|=smGetQueuedGetParamReturnValue(handle,paramVal1);
    smStat|=smGetQueuedGetParamReturnValue(handle,paramVal2);
    smStat|=smGetQueuedGetParamReturnValue(handle,paramVal3);

    return recordStatus(handle,smStat);
}

SM_STATUS smSetParameter( const smbus handle, const smaddr nodeAddress, const smint16 paramId, smint32 paramVal )
{
    smint32 nul;
    SM_STATUS smStat=0;

	if (smIsHandleOpen(handle) == smfalse) return SM_ERR_NODEVICE;
	
	smStat |= smAppendSetParamCommandToQueue(handle,paramId,paramVal);
	smStat |= smExecuteCommandQueue(handle,nodeAddress);
    smStat|=smGetQueuedSetParamReturnValue(  handle, &nul );

    return recordStatus(handle,smStat);
}

//accumulates status to internal variable by ORing the bits. returns same value that is fed as paramter
SM_STATUS recordStatus( const smbus handle, const SM_STATUS stat )
{
    //check if bus handle is valid & opened
    if(smIsHandleOpen(handle)==smfalse) return SM_ERR_NODEVICE;

    smBus[handle].cumulativeSmStatus|=stat;
    return stat;
}


/** This function returns all occurred SM_STATUS bits after smOpenBus or resetCumulativeStatus call*/
SM_STATUS getCumulativeStatus( const smbus handle )
{
    if(smIsHandleOpen(handle)==smfalse) return SM_ERR_NODEVICE;

    return smBus[handle].cumulativeSmStatus;
}

/** Reset cululative status so getCumultiveStatus returns 0 after calling this until one of the other functions are called*/
void resetCumulativeStatus( const smbus handle )
{
	if (smIsHandleOpen(handle) == smfalse) return;// SM_ERR_NODEVICE;

    smBus[handle].cumulativeSmStatus=0;
}

//------------------------------------------------------------------------------------------------------------------------------------------

#endif