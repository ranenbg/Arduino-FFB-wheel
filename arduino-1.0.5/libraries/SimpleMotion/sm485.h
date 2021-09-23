//VSD SM485 bus definitions

#ifndef SM485_H
#define SM485_H

//SERIAL COMMANDS
#define SM485_CRCINIT 0x0
#define SM485_BUFSIZE 128
#define SM485_RSBUFSIZE 128
#define SM485_CMDBUFSIZE 4096

//cmd number must be 0-31

#define SM485_VERSION 90
#define SM485_VERSION_COMPAT 85

#define SM485_MAX_PAYLOAD_BYTES 120

//00xb
#define SMCMD_MASK_0_PARAMS 0
//01xb, 2 bytes as payload
#define SMCMD_MASK_2_PARAMS 2
//10xb
#define SMCMD_MASK_N_PARAMS 4
//11xb
#define SMCMD_MASK_RESERVED 6
//xx1b
#define SMCMD_MASK_RETURN 1
//110b
#define SMCMD_MASK_PARAMS_BITS 6


#define SMCMD_ID(cmdnumber, flags) (((cmdnumber)<<3)|flags)

//format 5, u8 bytesfollowing,u8 toaddr,cmddata,u16 crc
//return, 6, u8 bytesfollowing,u8 fromaddr, returndata,u16 crc
#define SMCMD_INSTANT_CMD SMCMD_ID(4,SMCMD_MASK_N_PARAMS)
#define SMCMD_INSTANT_CMD_RET SMCMD_ID(4,SMCMD_MASK_N_PARAMS|SMCMD_MASK_RETURN)

//format  ID, u8 bytesfollowing,u8 toaddr,cmddata,u16 crc
//return, ID, u8 bytesfollowing,u8 fromaddr,returndata,u16 crc
#define SMCMD_BUFFERED_CMD SMCMD_ID(6,SMCMD_MASK_N_PARAMS)
#define SMCMD_BUFFERED_CMD_RET SMCMD_ID(6,SMCMD_MASK_N_PARAMS|SMCMD_MASK_RETURN)

//read return data from buffered cmds. repeat this command before SMCMD_BUFFERED_CMD until N!=120 from download
#define SMCMD_BUFFERED_RETURN_DATA SMCMD_ID(7,SMCMD_MASK_0_PARAMS)
#define SMCMD_BUFFERED_RETURN_DATA_RET SMCMD_ID(7,SMCMD_MASK_N_PARAMS|SMCMD_MASK_RETURN)

//cmd 20,u8 toaddr, u8 crc
//ret 21, u8=2, u8 fromaddr, u16 clock,u16 crc, muut clientit lukee t?n
//oldret 21, u8 fromaddr, u16 clock,u16 crc, muut clientit lukee t?n
#define SMCMD_GET_CLOCK SMCMD_ID(10,SMCMD_MASK_0_PARAMS)
#define SMCMD_GET_CLOCK_RET SMCMD_ID(10,SMCMD_MASK_2_PARAMS|SMCMD_MASK_RETURN)

#ifdef PROCESS_IMAGE_SUPPORT

//PROCESS_IMAGE communication not supported by SM V2.0.0. placeholders here

//upload process data to bus devices. each device knows it's data offset in payload
//(set by SMREGISTER_PDATA_IN_OFFSET, which is 0xffff if not accepting anything)
#define SMCMD_PROCESS_IMAGE SMCMD_ID(5,SMCMD_MASK_N_PARAMS)

//retrun of feedback process data is the tricky part
//the device that has process data address of 0 answers first including SMCMD header (3bytes) and then its own return process data
//SMREGISTER_PDATA_OUT_OFFSET is location of device's return processdata
#define SMCMD_PROCESS_IMAGE_RET SMCMD_ID(5,SMCMD_MASK_N_PARAMS|SMCMD_MASK_RETURN)

#endif


#endif
