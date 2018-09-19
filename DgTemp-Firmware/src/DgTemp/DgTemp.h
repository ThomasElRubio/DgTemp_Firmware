#ifndef _DGTEMP_h
#define _DGTEMP_h


#define INVERT_PIN				4
#define NON_INVERT_PIN			5
#define NUMBER_OF_SAMPLES 		504
#define SAMPLES_UNTIL_SETTLES	8
//#define newSample				false







#include <Arduino.h>
#include <SPI.h>
#include <DMAChannel.h>



extern volatile bool newSample;
extern volatile uint32_t code;
void ftmISR();
void spi1_isr();
void clockInit();
void timerInit();
void spiInit();
void moduleClockGateEnable();
void initClocks();
void initFlexTimer();
void initAdcClock();
void timerCounterEnable(bool Enable);
void initSpiBus();
void initDmaSpi();
void enableDmaInterrupt();







#endif