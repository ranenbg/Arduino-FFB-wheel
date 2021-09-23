
The is an Arduino library for the HX711 24-bit ADC for weight scales.
Data retrieval from the HX711 is done without halting the mcu, also on the 10SPS rate setting and with Multiple HX711's performing conversions simultaneously.
Tare function can also be performed without halting the mcu.
 
Filtering and smoothing: "Moving average" method from a rolling data set combined with removal of high/low outliers is used for the retrieved value.

Selectable values in the .h file:
- Moving average data set of 4, 8, 16, 32, 64 or 128 samples (default:16).
- Ignore high outlier; one sample is added to the data set, the peak high value of all samples in the data set is ignored (default:1)
- Ignore low outlier; one sample is added to the data set, the peak low value of all samples in the data set is ignored (default:1)

Caution: using a high number of samples will smooth the output value nicely but will also increase settling time and start-up/tare time (but not response time). It will also eat some memory.

The HX711 sample rate can be set to 10SPS or 80SPS (samples per second) by pulling pin 15 high (80SPS) or low (10SPS), ref HX711 data sheet.
On fabricated modules there is usually a solder jumper on the PCB for pin 15 high/low. The rate setting can be checked by measuring the voltage on pin 15.
ADC noise is worst on the 80SPS rate. Unless very quick settling time is required, 10SPS should be the best sample rate for most applications.

Start up and tare: from start-up/reset, the tare function seems to be more accurate if called after a "pre-warm-up" period running conversions continuously for a few seconds. See example files.

Measuring units: Adjust the scaling factor with the function setCalFactor(float) in setup, so that the "readable" output matches your preferred unit.

Hardware and ADC noise:
Wires between HX711 and load cell should be twisted and kept as short as possible.
Most available HX711 modules seems to follow the reference design, but be aware that some modules are poorly designed with under-sized capacitors, and noisy readings.
The Sparkfun module seems to differ from most other available modules as it has some additional components for noise reduction. 

Update 1.0.2:
- Implemented requested function setTareOffset(long newoffset)
- Initializing of the index counter in the class declaration: readIndex = 0
