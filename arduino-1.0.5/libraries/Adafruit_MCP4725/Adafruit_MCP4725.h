/**************************************************************************/
/*! 
    @file     Adafruit_MCP4725.h
    @author   K. Townsend (Adafruit Industries)
	@license  BSD (see license.txt)
	
	This is a library for the Adafruit MCP4725 breakout board
	----> http://www.adafruit.com/products/935
	
	Adafruit invests time and resources providing this open source code, 
	please support Adafruit and open-source hardware by purchasing 
	products from Adafruit!

	@section  HISTORY

    v1.0  - First release
	updated: Milos Rankovic (ranenbg@gmail.com), 09.03.2020.
*/
/**************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#define CMD_WRITEDAC            (0x40)  // Writes data to the DAC
#define CMD_WRITEDACEEPROM      (0x60)  // Writes data to the DAC and the EEPROM (persisting the assigned value after reset)
#define PWD_1KOHM				(0x42)  // 1KOhm to GND
#define PWD_100KOHM				(0x44)  // 100KOhm to GND
#define PWD_500KOHM				(0x46)  // 500KOhm to GND
#define DAT_ZERO				(0x00)  // dummy for data for power down commands

class Adafruit_MCP4725{
 public:
  Adafruit_MCP4725();
  void begin(uint8_t a);  
  void setVoltage( uint16_t output, bool writeEEPROM, uint8_t mode );

 private:
  uint8_t _i2caddr;
};
