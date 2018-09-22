#ifndef __ARDUINO_TEENSYDAC
#define __ARDUINO_TEENSYDAC




#include <Arduino.h>


class TeensyDAC {
	public:
		volatile uint16_t dacOutput();
		void dacSetup();
		void setOutput(uint16_t DCDC_OUTPUT);
		void enableDCDC();
		void disableDCDC();
	
	protected:
		static const uint32_t DAC_LOW_ADRESS = 0x400CC000;	// Address of the DAC value register (LSB)
		static const uint32_t DAC_HIGH_ADRESS = 0x400CC001;	// Address of the DAC value register (MSB)
		static const uint16_t DAC_INIT = 715;	//Roughly 0.58V DCDC Output of 0
		static const uint16_t DAC_MAX = 1380;	//Roughly 1.1V for maximum positive output of the DCDC converter
		static const uint16_t DAC_MIN = 150;	//Roughly 0.1V for maximum negative output of the DCDC-converter
	
		
};


#endif