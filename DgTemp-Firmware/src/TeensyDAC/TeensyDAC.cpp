#ifndef TEENSYDAC_CPP
#define TEENSYDAC_CPP



#include "TeensyDAC.h"

#define TEENSY_DAC A21
#define DCDC_ENABLE_PIN 19


uint16_t TeensyDAC::dacOutput(){
	this->DAC_DATL = *(volatile uint8_t *) this->DAC_LOW_ADRESS;
	this->DAC_DATH = *(volatile uint8_t *) this->DAC_HIGH_ADRESS;
	DAC_OUTPUT = ((DAC_DATH)<<8) | (DAC_DATL);
	return this->DAC_OUTPUT;
}


void TeensyDAC::dacSetup(){
	pinMode(DCDC_ENABLE_PIN,OUTPUT);
	digitalWrite(DCDC_ENABLE_PIN, LOW);
	pinMode(TEENSY_DAC,OUTPUT);
	analogWriteResolution(12);
	analogWrite(TEENSY_DAC, this->DAC_INIT);
	if(this->dacOutput() == DAC_INIT){
		this->enableDCDC();
	}
	else{
		this->setOutput(DAC_INIT);
		this->enableDCDC();
	}
}

void TeensyDAC::enableDCDC(){
	digitalWrite(DCDC_ENABLE_PIN,HIGH);
}
void TeensyDAC::disableDCDC(){
	digitalWrite(DCDC_ENABLE_PIN,LOW);
}

void TeensyDAC::setOutput(uint16_t DCDC_OUTPUT){
	if(DCDC_OUTPUT> DAC_MAX)	DCDC_OUTPUT = DAC_MAX;
	if(DCDC_OUTPUT<DAC_MIN)	DCDC_OUTPUT = DAC_MIN;
	analogWrite(TEENSY_DAC, DCDC_OUTPUT);
}

#endif