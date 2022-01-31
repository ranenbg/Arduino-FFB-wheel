#include "simplemotion.h"
#include <stdio.h>
#include <stdlib.h>

#define ADDR 255
#include "vsd_cmd.h"


//gcc -o testi *.c

//#include "D:\gd\vsdr\vsdr-stm32\src\simplemotion_defs.h"
#include "/home/tero/shr/svn/granited/firmware_design/eclipseworkspace/vsdr-stm32/src/simplemotion_defs.h"

smbus h;
       smint32 ret;
       SM_STATUS stat=0;



void test()
{
		//printf( "stat %d\n", smRawCommand( h, ADDR, 2, 0, &ret ) );
	//printf("ret=%08x\n",ret);

	//printf( "stat %d\n", smRawCommand( h, 2, 2, 0, &ret ) );
	//printf("ret=%08x\n",ret);


	
        //stat|=smAppendSetParamCommandToQueue(h,SMP_DIGITAL_OUT_VALUE_1,151);
//        stat|=smAppendSetParamCommandToQueue(h,SMP_INPUT_MULTIPLIER,11);
        stat|=smExecuteCommandQueue( h, ADDR );
        stat|=smGetQueuedSetParamReturnValue(h,&ret);
        printf("setstat=%d val=%d\n",stat,ret);

        stat=0;
        stat|=smAppendGetParamCommandToQueue(h,SMP_BUS_SPEED);
        stat|=smAppendGetParamCommandToQueue(h,SMP_SM_VERSION_COMPAT);
        stat|=smAppendGetParamCommandToQueue(h,SMP_DIGITAL_OUT_VALUE_1);
        stat|=smExecuteCommandQueue( h, ADDR );
        stat|=smGetQueuedGetParamReturnValue(h,&ret);
        printf("stat=%d val=%d\n",stat,ret);
        stat|=smGetQueuedGetParamReturnValue(h,&ret);
        printf("stat=%d val=%d\n",stat,ret);
        stat|=smGetQueuedGetParamReturnValue(h,&ret);
        printf("stat=%d val=%d\n",stat,ret);


/*        smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_RETURN_PARAM_LEN );
        smAppendSMCommandToQueue( h, SMPCMD_24B, SMPRET_32B );
        smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_RETURN_PARAM_ADDR );
        smAppendSMCommandToQueue( h, SMPCMD_32B, SMP_BUS_SPEED );

		smAppendSMCommandToQueue( h, SMPCMD_32B,SMP_INPUT_MULTIPLIER );
		smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_INPUT_MULTIPLIER );
		smAppendSMCommandToQueue( h, SMPCMD_24B, 100 );
//		SMP_INPUT_MULTIPLIER


        smExecuteCommandQueue( h, ADDR );
        printf("",smGetQueuedSMCommandReturnValue( h ));
        printf("",smGetQueuedSMCommandReturnValue( h ));
        printf("",smGetQueuedSMCommandReturnValue( h ));
        printf("",smGetQueuedSMCommandReturnValue( h ));
        printf("",smGetQueuedSMCommandReturnValue( h ));
        printf("",smGetQueuedSMCommandReturnValue( h ));
        printf("",smGetQueuedSMCommandReturnValue( h ));
 //       printf("%d\n",smGetQueuedSMCommandReturnValue( h ));*/

        /*	smAppendCommandToQueue( h, CMD_GET_PARAM, RUNTIME_FAULTBITS );
        smAppendCommandToQueue( h, CMD_GET_PARAM, RUNTIME_STATUSBITS );
        smAppendCommandToQueue( h, CMD_GET_PARAM, RUNTIME_FIRMWARE_VERSION );
        smExecuteCommandQueue( h, ADDR );
        printf("%d\n",smGetQueuedCommandReturnValue( h, 0 ));
        printf("%d\n",smGetQueuedCommandReturnValue( h, 1 ));
        printf("%d\n",smGetQueuedCommandReturnValue( h, 2 ));
        */
}

void tests();

/*BL functions started by writing a value to this param:
 * 1:mass erase and reset write pos counter to 0
 * 2:write block
 * 3:verify flash (update BOOTLOADER_STAT_FLASH_VERIFIED_OK status)
 * 4:start main app
 */
#define SMP_BOOTLOADER_FUNCTION 191
//upload 16bits of data to buffer. max buffer length 4096 bytes before it must be written with "write block" function
//total uploaded amount must be multiple of 4 bytes before issuing write function
#define SMP_BOOTLOADER_UPLOAD 192
//bootloaded status
#define SMP_BOOTLOADER_STAT 193
	#define BOOTLOADER_STAT_FLASH_VERIFIED_OK BV(0)
#define SMP_FIRMWARE_VERSION 6010
#define SMP_BACKWARDS_COMP_VERSION 6012

#define SUPPORTED_BL_VERSION_MIN 10000
#define SUPPORTED_BL_VERSION_MAX 10010


//load app rom from file. args:
//pass pointer to "data". this func will allocate the memory
//return number of 16bit words or -1 if load failed
int loadBLfile( const char *filename, smuint16 **data )
{
	int bytes;
	int i;
	FILE *f;
	smuint16 *buf;
	
	//open file
	f=fopen(filename,"rb");
	if(f==NULL)
		return -1;
	
	//find file size
	fseek(f, 0L, SEEK_END);
	bytes = ftell(f);
	fseek(f, 0L, SEEK_SET);
	printf("size=%d\n",bytes);
	//allocate
	buf=(smuint16*)malloc((bytes+1)/2);//round upwards
	if(buf==NULL)
		return -1;
	
	//load
	for(i=0;i<bytes;i++)
	{
		int get=fgetc(f);
		if(get==EOF) 
			return -1;
		
		//treat data as byte array
		((smuint8*)buf)[i]=get;
	}
		printf(".\n");
	//fclose(f);
		printf(".\n");
	
	*data=buf;
	
	return ((bytes+1)/2);
}

#define BL_CHUNK_LEN 32

int uploadGCfile(const char *fname)
{
	smint32 blversion,blcompversion;
	stat=0;	
	smuint16 *bin;
	int i,c;
	
	int size=loadBLfile(fname,&bin);
	if(size<0)
	{
		printf("load failed\n");
		return;
	}
	printf("loaded %d words\n",size);
	
	
	
	//get possible node detected faults in communications
	stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_RETURN_PARAM_ADDR );
	stat|=smAppendSMCommandToQueue( h, SMPCMD_24B, SMP_CUMULATIVE_STATUS );
	//set return param type and command type
	stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_RETURN_PARAM_LEN );
	stat|=smAppendSMCommandToQueue( h, SMPCMD_32B, SMPRET_CMD_STATUS );
	stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_BOOTLOADER_UPLOAD );
	stat|=smExecuteCommandQueue( h, ADDR );
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);
	int smnodestats=ret;
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);	
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);	
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);	
	
	if(stat!=SM_OK || smnodestats!=SMP_CMD_STATUS_ACK)
	{
		printf("SM errors occurred:%d\n",smnodestats);
		exit(5);
	}
	
	//upload data in 32=BL_CHUNK_LEN word chunks
	for(i=0;i<size;)
	{
		printf("pos %d\n",i);
		//getchar();
		
		stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_BOOTLOADER_UPLOAD );
		for(c=0;c<BL_CHUNK_LEN;c++)
		{
			smuint16 upword;
			//pad end of file with constant
			if(i>=size)
				upword=0xeeee;
			else
				upword=bin[i];
			printf("i=%d\n",i);
			
			stat|=smAppendSMCommandToQueue( h, SMPCMD_24B, upword );
			i++;
		}
//		stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_BOOTLOADER_FUNCTION );
//		stat|=smAppendSMCommandToQueue( h, SMPCMD_24B, 2);//do write
		
		stat|=smExecuteCommandQueue( h, ADDR );
		
		for(c=0;c<BL_CHUNK_LEN+1;c++)
		{
			stat|=smGetQueuedSMCommandReturnValue( h,&ret );
			if(stat!=SM_OK)
			{
				printf("error stat=%d\n",stat);
				exit(6);
			}
		}
	}
	
	printf("done. enter to flash\n");
	getchar();
	stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_BOOTLOADER_FUNCTION );
	stat|=smAppendSMCommandToQueue( h, SMPCMD_24B, 5);//flash gc
	stat|=smExecuteCommandQueue( h, ADDR );
}

void blcommtest(const char *fname)
{
	smint32 blversion,blcompversion;
	stat=0;	
	smuint16 *bin;
	int i,c;
	
	int size=loadBLfile(fname,&bin);
	if(size<0)
	{
		printf("load failed\n");
		return;
	}
	printf("loaded %d words\n",size);
		
	
	//read bl variables
	stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_RETURN_PARAM_LEN );
	stat|=smAppendSMCommandToQueue( h, SMPCMD_32B, SMPRET_32B );
	stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_RETURN_PARAM_ADDR );
	stat|=smAppendSMCommandToQueue( h, SMPCMD_32B, SMP_FIRMWARE_VERSION );
	stat|=smAppendSMCommandToQueue( h, SMPCMD_32B, SMP_BACKWARDS_COMP_VERSION );
	stat|=smExecuteCommandQueue( h, ADDR );
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);
	blversion=ret;
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);
	blcompversion=ret;
	
	//if version ok
	//if(blversion

	//mass erase flash
	stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_BOOTLOADER_FUNCTION );
	stat|=smAppendSMCommandToQueue( h, SMPCMD_32B, 1 );//do mass erase
	stat|=smExecuteCommandQueue( h, ADDR );
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);

	usleep(1000000);
	
	//flash
	
	//get possible node detected faults in communications
	stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_RETURN_PARAM_ADDR );
	stat|=smAppendSMCommandToQueue( h, SMPCMD_24B, SMP_CUMULATIVE_STATUS );
	//set return param type and command type
	stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_RETURN_PARAM_LEN );
	stat|=smAppendSMCommandToQueue( h, SMPCMD_32B, SMPRET_CMD_STATUS );
	stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_BOOTLOADER_UPLOAD );
	stat|=smExecuteCommandQueue( h, ADDR );
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);
	int smnodestats=ret;
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);	
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);	
	stat|=smGetQueuedSMCommandReturnValue( h,&ret );printf("stat=%d val=%d\n",stat,ret);	
	
	if(stat!=SM_OK || smnodestats!=SMP_CMD_STATUS_ACK)
	{
		printf("SM errors occurred:%d\n",smnodestats);
		exit(5);
	}
	
	//upload data in 32=BL_CHUNK_LEN word chunks
	for(i=0;i<size;)
	{
		printf("pos %d\n",i);
		//getchar();
		
		stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_BOOTLOADER_UPLOAD );
		for(c=0;c<BL_CHUNK_LEN;c++)
		{
			smuint16 upword;
			//pad end of file with constant
			if(i>=size)
				upword=0xeeee;
			else
				upword=bin[i];
			printf("i=%d\n",i);
			
			stat|=smAppendSMCommandToQueue( h, SMPCMD_24B, upword );
			i++;
		}
		stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_BOOTLOADER_FUNCTION );
		stat|=smAppendSMCommandToQueue( h, SMPCMD_24B, 2);//do write
		
		stat|=smExecuteCommandQueue( h, ADDR );
		
		for(c=0;c<BL_CHUNK_LEN+3;c++)
		{
			stat|=smGetQueuedSMCommandReturnValue( h,&ret );
			if(stat!=SM_OK)
			{
				printf("error stat=%d\n",stat);
				exit(6);
			}
		}
	}
	
	printf("done. enter to run\n");
	getchar();
	stat|=smAppendSMCommandToQueue( h, SMPCMD_SETPARAMADDR, SMP_BOOTLOADER_FUNCTION );
	stat|=smAppendSMCommandToQueue( h, SMPCMD_24B, 4);//launch
	stat|=smExecuteCommandQueue( h, ADDR );

	
}

int main(int args, const char *argv[])
{
	if(args<2)
	{
		printf("args: rom bin file name\n");
		return(0);
	}

	smSetDebugOutput(High,stderr);

	smSetTimeout(1500);
        h=smOpenBus("COM17"); //17=USB1 16=0

	printf("handle=%d\n",h);
	if(h==-1) return 3;

	
	uploadGCfile(argv[1]);
	//blcommtest(argv[1]);

	smCloseBus(h);
}
