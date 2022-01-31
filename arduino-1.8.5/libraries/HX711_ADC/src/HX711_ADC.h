//-------------------------------------------------------------------------------------
// HX711_ADC.h
// Arduino master library for HX711 24-Bit Analog-to-Digital Converter for Weigh Scales
// Olav Kallhovd sept2017
// Tested with      : HX711 asian module on channel A and YZC-133 3kg load cell
// Tested with MCU	: Arduino Nano
// modified by: Milos Rankovic 14.12.2021. - set only 1 sample for fastest conversion
//-------------------------------------------------------------------------------------

#ifndef HX711_ADC_h
#define HX711_ADC_h

#include <Arduino.h>

#define SAMPLES 			1									// no of samples in moving average data set, value must be 1, 2, 4, 8, 16, 32, 64, 128 or 256
#define IGN_HIGH_SAMPLE 	0 									// adds one sample to the set and ignore peak high sample, value must be 0 or 1
#define IGN_LOW_SAMPLE 		0 									// adds one sample to the set and ignore peak low sample, value must be 0 or 1

#define DATA_SET 	SAMPLES + IGN_HIGH_SAMPLE + IGN_HIGH_SAMPLE // total samples in memory

#if (SAMPLES  != 1) & (SAMPLES  != 2) & (SAMPLES  != 4) & (SAMPLES  != 8) & (SAMPLES  != 16) & (SAMPLES  != 32) & (SAMPLES  != 64) & (SAMPLES  != 128) & (SAMPLES  != 256)
	#error "number of SAMPLES not valid!"
#endif

#if (SAMPLES == 1)
#define DIVB 0
#elif (SAMPLES == 2)
#define DIVB 1
#elif (SAMPLES == 4)
#define DIVB 2
#elif  (SAMPLES == 8)
#define DIVB 3
#elif  (SAMPLES == 16)
#define DIVB 4
#elif  (SAMPLES == 32)
#define DIVB 5
#elif  (SAMPLES == 64)
#define DIVB 6
#elif  (SAMPLES == 128)
#define DIVB 7
#elif  (SAMPLES == 256)
#define DIVB 8
#endif

class HX711_ADC
{	
		
	public:
		HX711_ADC(uint8_t dout, uint8_t sck); 	    // constructor
		void setGain(uint8_t gain = 128); 			// value should be 32, 64 or 128*
		void begin();
		void begin(uint8_t gain);
		void start(unsigned int t); 				// start and tare one HX711
		int startMultiple(unsigned int t); 			// start and tare multiple HX711 simultaniously
		void tare(); 								// zero the scale, wait for tare to finnish
		void tareNoDelay(); 						// zero the scale, do tare in loop without waiting for tare to finnish
		void setCalFactor(float cal); 				// calibration factor, raw data is divided by this value to convert to readable data
		float getCalFactor(); 						// returns the current calibration factor
		float getData(); 							// returns data from the moving average data set 
		float getSingleConversion(); 				// for testing
		void powerDown(); 
		void powerUp(); 
		bool getTareStatus();						// returns 1 if tareNoDelay() operation is complete
		long getTareOffset();
		void setTareOffset(long newoffset);
		uint8_t update(); 							// if conversion is ready; read out 24 bit data and add to data set

	protected:
		uint8_t conversion24bit(); 					// if conversion is ready: returns 24 bit data and starts the next conversion
		long smoothedData();
		uint8_t sckPin; 							// HX711 pd_sck pin
		uint8_t doutPin; 							// HX711 dout pin
		uint8_t GAIN;
		float calFactor;
		long dataSampleSet[DATA_SET + 1]; 			// data set, make voltile if interrupt is used 
		long tareOffset;
		int readIndex = 0;
		long timeStamp;
		uint8_t isFirst = 1;
		uint8_t tareTimes;
		const uint8_t divBit = DIVB;
		bool doTare;
		bool startStatus;
		uint8_t convRslt;
		bool tareStatus;
};	

#endif
   