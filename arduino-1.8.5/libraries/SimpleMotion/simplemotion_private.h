//Internal functions & definitions, not for library user
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

#ifndef SIMPLEMOTION_PRIVATE_H
#define SIMPLEMOTION_PRIVATE_H

#include "simplemotion.h"
#include "busdevice.h"
#ifndef __Arduino__
#include <stdio.h>
#endif

#define SM_VERSION 0x020001
//max number of simultaneously opened buses. change this and recompiple SMlib if
//necessary (to increase channels or reduce to save memory)
#define SM_MAX_BUSES 1

//bus device types
#define BUSDEV_NONE 0
#define BUSDEV_RS 1 /* rs232 like com port support */
#define BUSDEV_FTDI 2 /*not implemented yet: direct FTDI lib support*/

#define SM_BUSDEVICENAME_LEN 64

//default timeout in ms
//Argon drive's worst case response time should be ~20ms with max length packets
#define SM_READ_TIMEOUT 500


extern const smuint8 table_crc16_hi[];
extern const smuint8 table_crc16_lo[];
extern FILE *smDebugOut; //such as stderr or file handle. if NULL, debug info disbled
extern smuint16 readTimeoutMs;

void smDebug( smbus handle, smVerbosityLevel verbositylevel, char *format, ...);

SM_STATUS smRawCmd( const char *axisname, smuint8 cmd, smuint16 val, smuint32 *retdata );
SM_STATUS recordStatus(const smbus handle,const SM_STATUS stat);

typedef struct {
        /* ID=0 param size 30 bits (cmd total 4 bytes)
         * ID=1 param size 22 bits (cmd total 3 bytes)
         * ID=2 set parameter store address, param: 14 bits=register address  (cmd total 2 bytes)
         * ID=3 reserved
         */
        long param :30; //LSB 30 bits
        long ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommand32;

typedef struct {
        /* ID=0 param size 30 bits (cmd total 4 bytes)
         * ID=1 param size 22 bits (cmd total 3 bytes)
         * ID=2 set parameter store address, param: 14 bits=register address  (cmd total 2 bytes)
         * ID=3 reserved
         */
        long param :14; //LSB 30 bits
        long ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommand16;

typedef struct {
        /* ID=0 param size 30 bits (cmd total 4 bytes)
         * ID=1 param size 22 bits (cmd total 3 bytes)
         * ID=2 set parameter store address, param: 14 bits=register address  (cmd total 2 bytes)
         * ID=3 reserved
         */
        long param :22; //MSB 30 bits
        long ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommand24;

//SM payload command return data structure
typedef struct {
        /* ID=0 ret data 30 bits (tot 4 bytes)
         * ID=1 ret data 22 bits (tot 3 bytes)
         * ID=2 ret data 16 bits (tot 2 bytes), i.e. ACK/NACK.
         * ID=3 reserved
         */
        long retData: 30; //LSB 30 bits
        long ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommandRet32;
//SM payload command return data structure
typedef struct {
        /* ID=0 ret data 30 bits (tot 4 bytes)
         * ID=1 ret data 22 bits (tot 3 bytes)
         * ID=2 ret data 16 bits (tot 2 bytes), i.e. ACK/NACK.
         * ID=3 reserved
         */
        long retData: 22; //LSB 30 bits
        long ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommandRet24;
//SM payload command return data structure
typedef struct {
        /* ID=0 ret data 30 bits (tot 4 bytes)
         * ID=1 ret data 22 bits (tot 3 bytes)
         * ID=2 ret data 16 bits (tot 2 bytes), i.e. ACK/NACK.
         * ID=3 reserved
         */
        long retData: 14; //LSB 30 bits
        long ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommandRet16;
//SM payload command return data structure
typedef struct {
        /* ID=0 ret data 30 bits (tot 4 bytes)
         * ID=1 ret data 22 bits (tot 3 bytes)
         * ID=2 ret data 16 bits (tot 2 bytes), i.e. ACK/NACK.
         * ID=3 reserved
         */
        long retData: 6; //LSB 30 bits
        long ID:2; //MSB 2 bits. when serailzied to bytestream byte4 must be transmitted first to contain ID
} __attribute__ ((packed)) SMPayloadCommandRet8;

#endif // SIMPLEMOTION_PRIVATE_H
