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












static volatile bool newSample;
static volatile uint32_t code;

//void clockInit();
//void timerInit();
void spiInit();
uint32_t getCode();
bool sampleReceived();
void waitForSample();




//void moduleClockGateEnable();
//void initClocks();
//void initFlexTimer();
//static inline void initAdcClock();
//void timerCounterEnable(bool Enable);
//void initSpiBus();
void initDmaSpi();
void enableDmaInterrupt();
//Interrupt-Service-Routines
void ftmISR();
void spi1ISR();










static inline void initSpiBus(){ 	
	SPI1.begin();
	SPI1.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
	SPI1_CTAR1 |= 1<<2;
	SPI1_CTAR1 &= ~(1<<1);
	SPI1_CTAR1 |= 1;
	SPI1_CTAR1 |= 1<<16;
	//Setting delay between Interrupt an SCK and the delay between the transmission of two words to ensure a right timing
	SPI1_CTAR1 &= ~(0b11<<22);
	SPI1_CTAR1 &= ~(0b11<<20);
	SPI1_CTAR1 &= ~(0b11<<18);
	SPI1_CTAR1 &= ~(0b1111<<12);
	SPI1_CTAR1 &= ~(0b1111<<8);
	SPI1_CTAR1 &= ~(0b1111<<4);
	SPI1_CTAR1 |= 0b0100<<12;
	//SPI1_CTAR1 |= 0b0010<<4;
	SPI1_CTAR1 |= 0b01<<20;
	SPI1_CTAR1 |= 0b10<<18;
} 




class DgTemp{
	public:
		void clockInit();
		void timerInit();
		bool receivedSample();
		void waitForSample();
		
	private:
		
		static volatile uint32_t code;
		static volatile bool newSample;
		static volatile uint32_t recData[2];
		static const uint32_t CLEAR_FLAGS[];
		static const uint32_t data[];
		static void moduleClockGateEnable();
		static void initClocks();
		static void initFlexTimer();
		static void ftm_ISR(void);
		static void initAdcClock();
		static void timerCounterEnable(bool Enable);
		static void initSpiBus();
		
		
	protected:
		static const uint32_t SPI_RESUME_TRANSACTION = 0b1 << 28; 
		static const uint32_t SPI_END_TRANSACTION = 0b11 << 27;
		
		
		
		//static const uint32_t data[0] = SPI_RESUME_TRANSACTION;
		//static const uint32_t data[1] = SPI_END_TRANSACTION;
};








#endif