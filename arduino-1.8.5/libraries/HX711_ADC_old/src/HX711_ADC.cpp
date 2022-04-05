//-------------------------------------------------------------------------------------
// HX711_ADC.h
// Arduino master library for HX711 24-Bit Analog-to-Digital Converter for Weigh Scales
// Olav Kallhovd sept2017
// Tested with      : HX711 asian module on channel A and YZC-133 3kg load cell
// Tested with MCU	: Arduino Nano
//-------------------------------------------------------------------------------------

#include <Arduino.h>
#include <HX711_ADC.h>


HX711_ADC::HX711_ADC(uint8_t dout, uint8_t sck) //constructor
{ 	
	doutPin = dout;
	sckPin = sck;
} 

void HX711_ADC::setGain(uint8_t gain)  //value should be 32, 64 or 128*
{
	if(gain < 64) GAIN = 2; //32, channel B
	else if(gain < 128) GAIN = 3; //64, channel A
	else GAIN = 1; //128, channel A
}

void HX711_ADC::begin()
{
	pinMode(sckPin, OUTPUT);
	pinMode(doutPin, INPUT);
	setGain(128);
	powerUp();
}


void HX711_ADC::begin(uint8_t gain)
{
	pinMode(sckPin, OUTPUT);
	pinMode(doutPin, INPUT);
	setGain(gain);
	powerUp();
}

/*  start(t): will do conversions continuously for 't' +400 milliseconds (400ms is min. settling time at 10SPS). 
*   Running this for 1-5s before tare() seems to improve the tare accuracy */
void HX711_ADC::start(unsigned int t)
{
	t += 50;
	while(millis() < t) {
		getData();
	}
	tare();
	tareStatus = 0;
}	

int HX711_ADC::startMultiple(unsigned int t)
{
	if(startStatus == 0) {
		if(isFirst) {
			t += 50; //min time for HX711 to be stable
			timeStamp = millis();
			isFirst = 0;
		}	
		if(millis() < timeStamp + t) {
			//update(); //do conversions during stabi time
			update();
			return 0;
		}
		else { //do tare after stabi time
			doTare = 1;
			update();
			if(convRslt == 2) {	
				doTare = 0;
				convRslt = 0;
				startStatus = 1;
			}
		}
	}
	return startStatus;
}

void HX711_ADC::tare() 
{
	uint8_t rdy = 0;
	doTare = 1;
	tareTimes = 0;
	while(rdy != 2) {
		rdy = update();
	}
}

bool HX711_ADC::getTareStatus() 
{
	bool t = tareStatus;
	tareStatus = 0;
	return t;
}

void HX711_ADC::tareNoDelay() 
{
	doTare = 1;
	tareTimes = 0;
}

void HX711_ADC::setCalFactor(float cal) //raw data is divided by this value to convert to readable data
{
	calFactor = cal;
}

float HX711_ADC::getCalFactor() //raw data is divided by this value to convert to readable data
{
	return calFactor;
}

//call update() in loop
//if conversion is ready; read out 24 bit data and add to data set, returns 1
//if tare operation is complete, returns 2
//else returns 0
uint8_t HX711_ADC::update() 
{
	//#ifndef USE_PC_INT
	byte dout = digitalRead(doutPin); //check if conversion is ready
	if (!dout) {
		conversion24bit();
		//if(s) Serial.print(s);
	}
	else convRslt = 0;
	return convRslt;
	//#endif
}

float HX711_ADC::getData() // return fresh data from the moving average data set
{
	long k = 0;
	long data = 0;
	data = smoothedData() - tareOffset;
	data = (data >> divBit);
	float x = (float)data / calFactor;
	return x;
}

long HX711_ADC::smoothedData() 
{
	long data = 0;
	long L = 0xFFFFFF;
	long H = 0x00;
	for (uint8_t r = 0; r < DATA_SET; r++) {
		if (L > dataSampleSet[r]) L = dataSampleSet[r]; // find lowest value
		if (H < dataSampleSet[r]) H = dataSampleSet[r]; // find highest value
		data += dataSampleSet[r];
	}
	if(IGN_LOW_SAMPLE) data -= L; //remove lowest value
	if(IGN_HIGH_SAMPLE) data -= H; //remove highest value
	return data;
}

uint8_t HX711_ADC::conversion24bit()  //read 24 bit data and start the next conversion
{
	unsigned long data = 0;
	uint8_t dout;
	convRslt = 0;
	for (uint8_t i = 0; i < (24 + GAIN); i++) { //read 24 bit data + set gain and start next conversion
		delayMicroseconds(1); // required for faster mcu's?
		digitalWrite(sckPin, 1);
		delayMicroseconds(1); 
		if (i < (24)) {
			dout = digitalRead(doutPin);
			data = data << 1;
			if (dout) {
			data++;
			}
		}
		digitalWrite(sckPin, 0);
	}
	data = data ^ 0x800000; // if out of range (min), change to 0
	if (readIndex == DATA_SET - 1) {
		readIndex = 0;
	}
	else {
		readIndex++;
	}
	if(data > 0)  {
		convRslt++;
		dataSampleSet[readIndex] = (long)data;
		if(doTare) {
			if (tareTimes < DATA_SET) {
			tareTimes++;
			}
			else {
				tareOffset = smoothedData();
				tareTimes = 0;
				doTare = 0;
				tareStatus = 1;
				convRslt++;
			}
		}
	}
}

void HX711_ADC::powerDown() 
{
	digitalWrite(sckPin, LOW);
	digitalWrite(sckPin, HIGH);
}

void HX711_ADC::powerUp() 
{
	digitalWrite(sckPin, LOW);
}

//get the tare offset (raw data value output without the scale "CalFactor")
long HX711_ADC::getTareOffset() 
{
	return tareOffset;
}

//set new tare offset (raw data value input without the scale "CalFactor")
void HX711_ADC::setTareOffset(long newoffset)
{
	tareOffset = newoffset;
}

//for testing only:
//if ready: returns 24 bit data and starts the next conversion, else returns -1
float HX711_ADC::getSingleConversion()  
{	
	long data = 0;
	byte dout = digitalRead(doutPin); //check if conversion is ready
	if (!dout) {
		//data = conversion24bit();
		data = data - tareOffset;
		float x = (float) data/calFactor;
		return x;
	}
	else return -1;
}
