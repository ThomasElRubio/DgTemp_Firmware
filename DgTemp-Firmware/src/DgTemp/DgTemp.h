#ifndef __ARDUINO_DGTEMP
#define __ARDUINO_DGTEMP


#include <Arduino.h>
#include <SPI.h>
#include <DMAChannel.h>

#define INVERT_PIN			4
#define NON_INVERT_PIN			5
#define NUMBER_OF_SAMPLES 		504
#define SAMPLES_UNTIL_SETTLES	8

volatile uint32_t recData[] = {0, 0};
volatile bool newSample = false;
volatile uint32_t code;
const uint32_t SPI_RESUME_TRANSACTION = 0b1 << 28; 
const uint32_t SPI_END_TRANSACTION = 0b11 << 27;
const uint32_t CLEAR_FLAG[] = {0xFF0F0000};
const uint32_t data[] = {SPI_RESUME_TRANSACTION,SPI_END_TRANSACTION};


DMAChannel tx;
DMAChannel rx;
DMAChannel Trigger;

void ftmISR();
void spi1_isr();

class DgTempClass {
	public:
		void clockInit();
		void timerInit();
		void spiInit();
		
		
		
		
	protected:
		void moduleClockGateEnable();
		void initClocks();
		void initFlexTimer();
		void initAdcClock();
		void timerCounterEnable(bool Enable);
		void initSpiBus();
		void initDmaSpi();
		void enableDmaInterrupt();
		
};






#endif