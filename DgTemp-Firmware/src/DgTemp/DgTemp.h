#ifndef _DGTEMP_H
#define _DGTEMP_H


#define INVERT_PIN				4
#define NON_INVERT_PIN			5
#define NUMBER_OF_SAMPLES 		504
#define SAMPLES_UNTIL_SETTLES	8
//#define newSample				false







#include <Arduino.h>
#include <SPI.h>
#include <DMAChannel.h>
#include <stdint.h>



class DgTemp{
	public:
		void clockInit();
		void timerInit();
		void spiInit();
		bool receivedSample();
		void waitForSample();
		uint32_t getCode();
		
	private:
		static DMAChannel tx;
		static DMAChannel rx;
		static DMAChannel Trigger;
		static volatile uint32_t code;
		static volatile bool newSample;
		static volatile uint32_t recData[2];
		static const uint32_t CLEAR_FLAGS[];
		static const uint32_t data[];
		static void moduleClockGateEnable();
		static void initClocks();
		static void initFlexTimer();
		static void ftm_ISR(void);
		static void spi1ISR(void);
		static void initAdcClock();
		static void timerCounterEnable(bool Enable);
		static void initSpiBus();
		static void initDmaSpi();
		static void enableDmaInterrupt();
		
		
	protected:
		static const uint32_t SPI_RESUME_TRANSACTION = 0b1 << 28; 
		static const uint32_t SPI_END_TRANSACTION = 0b11 << 27;
};








#endif