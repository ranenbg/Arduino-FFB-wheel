#ifndef HX711_h
#define HX711_h

#include "Arduino.h"
#include <digitalWriteFast.h>

#define PD_SCK				5//SCK
#define PD_OUT				4//MISO

#define NB_LC_CHANNELS		2

class HX711
{
private:
	// 		u8 PD_SCK;		// Power Down and Serial Clock Input Pin
	// 		u8 DOUT;		// Serial Data Output Pin
		u8 mGain;		// amplification factor
		u8 mGainChan0;		// amplification factor
public:
		s32 mOffset[NB_LC_CHANNELS];	// used for tare weight
		f32 mScale[NB_LC_CHANNELS];		// used to return weight in grams, kg, ounces, whatever

	public:
		// define clock and data pin, channel, and gain factor
		// channel selection is made by passing the appropriate gain: 128 or 64 for channel A, 32 for channel B
		// gain: 128 or 64 for channel A; channel B works with 32 gain factor only
		HX711(u8 dout,u8 pd_sck,u8 gain = 128);

		virtual ~HX711();

		// check if HX711 is ready
		// from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
		// input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
		b8 is_ready()			{ return digitalReadFast(PD_OUT) == LOW; }

		// set the gain factor; takes effect only after a call to read()
		// channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
		// depending on the parameter, the channel is also set to either A or B
		void set_gain (u8 channel,u8 gain = 128);

		void set_channel (u8 channel);

		// waits for the chip to be ready and returns a reading
		s32 read (u8 next_channel=0);

		// returns an average reading; times = how many times to read
		s32 read_average(u8 times = 10);

		s32 get_value(u8 channel,u8 times = 1)		{ return read_average(times) - mOffset[channel]; }
		f32 get_units(u8 channel,u8 times = 1)		{ return get_value(channel,times) / mScale[channel]; }
		void tare(u8 times = 10);
		void set_scale(u8 channel,f32 scale)	{ mScale[channel] = scale; }
		void set_offset(u8 channel,s32 offset)	{ mOffset[channel] = offset; }
		void power_down()				{ digitalWrite(PD_SCK,LOW); digitalWrite(PD_SCK,HIGH); delayMicroseconds(100); }
		void power_up()					{ digitalWrite(PD_SCK,LOW); }
};

#endif /* HX711_h */