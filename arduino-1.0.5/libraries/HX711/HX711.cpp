#include <Arduino.h>
#include <HX711.h>

#define NOP() __asm__ __volatile__ ("nop\n\t")

HX711::HX711(u8 dout, u8 pd_sck, u8 gain) 
{
// 	PD_SCK 	= pd_sck;
// 	DOUT 	= dout;
	
	pinMode(PD_SCK, OUTPUT);
	pinMode(PD_OUT, INPUT);

	for (u8 i = 0; i < NB_LC_CHANNELS; i++)
	{
		mScale[i] = 1.0f;
		mOffset[i] = 0;
	}
	mGainChan0 = 1;
	set_gain(gain);
}

HX711::~HX711() 
{
}

void HX711::set_gain(u8 channel,u8 gain)
{
	if (channel == 0)
	switch (gain)
	{
	case 64:		// channel A, gain factor 64
		mGainChan0 = 3;
		break;
	default:
	case 128:		// channel A, gain factor 128
		mGainChan0 = 1;
		break;
	}
	set_channel(channel);
}

void HX711::set_channel(u8 channel)
{
	mGain = (channel == 1) ? 2 : mGainChan0;
	digitalWriteFast(PD_SCK,LOW);
	read();
}

#define SCK_HIGH()	{digitalWriteFast(PD_SCK, HIGH);NOP(); NOP(); NOP(); NOP();}
#define SCK_LOW()	{digitalWriteFast(PD_SCK, LOW);NOP(); NOP(); NOP(); NOP();}

s32 HX711::read (u8 next_channel)
{
	// wait for the chip to become ready
	while (!is_ready());

	u8 data[3];

	// pulse the clock pin 24 times to read the data
	for (u8 j = 3; j--;) 
	{
		for (char i = 8; i--;) 
		{
			SCK_HIGH();
			bitWrite(data[j],i,digitalReadFast(PD_OUT));
			SCK_LOW();
		}
	}

	// set the channel and the gain factor for the next reading using the clock pin
	u8 gain_chan = mGain;
	if (next_channel == 1)
		gain_chan = 2;
	for (u8 i = 0; i < gain_chan; i++) 
	{
		SCK_HIGH();
		SCK_LOW();
	}

	data[2] ^= 0x80;

	return ((u32)data[2] << 16) | ((u32)data[1] << 8) | (u32)data[0];
}

s32 HX711::read_average (u8 times) 
{
	s32 sum = 0;
	for (u8 i = 0; i < times; i++)
		sum += read();
	return sum / times;
}

void HX711::tare(u8 times)
{
	for (u8 i = 0; i < NB_LC_CHANNELS; i++)
	{
		set_channel(i);
		s32 sum = read_average(times);
		set_offset(i,sum);
	}
	set_channel(0);
}
