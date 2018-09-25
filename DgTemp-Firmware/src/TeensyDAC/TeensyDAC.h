#ifndef TEENSYDAC_H
#define TEENSYDAC_H




#include <Arduino.h>


class TeensyDAC {
	public:
		uint16_t dacOutput();
		void dacSetup();
		void setOutput(uint16_t DCDC_OUTPUT);
		void enableDCDC();
		void disableDCDC();
	private:
		uint16_t DAC_OUTPUT;
		uint8_t	DAC_DATL;
		uint8_t DAC_DATH;
	
	protected:
		static const uint32_t DAC_LOW_ADRESS = 0x400CC000;	// Address of the DAC value register (LSB)
		static const uint32_t DAC_HIGH_ADRESS = 0x400CC001;	// Address of the DAC value register (MSB)
		static const uint16_t DAC_INIT = 2048;	//Roughly 0.58V DCDC Output of 0
		static const uint16_t DAC_MAX = 4095;	//Roughly 1.1V for maximum positive output of the DCDC converter
		static const uint16_t DAC_MIN = 0;	//Roughly 0.1V for maximum negative output of the DCDC-converter
	
		
};


#endif