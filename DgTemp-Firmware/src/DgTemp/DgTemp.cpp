#ifndef DGTEMP_CPP
#define DGTEMP_CPP


#include "DgTemp.h"
#include <Arduino.h>
#include <SPI.h>
#include <DMAChannel.h>




DMAChannel tx;
DMAChannel rx;
DMAChannel Trigger;

volatile uint32_t recData[] = {0, 0};
static const uint32_t SPI_RESUME_TRANSACTION = 0b1 << 28; 
static const uint32_t SPI_END_TRANSACTION = 0b11 << 27;
static const uint32_t CLEAR_FLAG[] = {0xFF0F0000};
static const uint32_t data[] = {SPI_RESUME_TRANSACTION,SPI_END_TRANSACTION};


uint32_t getCode(){
	return code;
}

bool sampleReceived(){
	return newSample;
}

void waitForSample(){
	newSample = false;
}


void clockInit(){
	moduleClockGateEnable();
	initClocks();
	moduleClockGateEnable();
}

void spiInit(){
	initSpiBus();
	initDmaSpi();
}

void timerInit(){
	initFlexTimer();
	initAdcClock();
	timerCounterEnable(true);
}



void ftmISR(void){
	FTM0_STATUS;                                        // To clear FTM channel interrupt the FTM0_Satus register must be read and cleared by writing a 0 to it afterwards
	FTM0_STATUS = 0;                                    // Writing a 0 to the FTM0_STATUS register clears all interrupts on all channels
	//GPIOA_PTOR |= (!(SPI1_TCR % (2*(NUMBER_OF_SAMPLES + SAMPLES_UNTIL_SETTLES)<<16)))<<13;
	//GPIOD_PTOR |= (!(SPI1_TCR % (2*(NUMBER_OF_SAMPLES + SAMPLES_UNTIL_SETTLES)<<16)))<<7;
}

void spi1ISR(void) {
  rx.clearInterrupt();
  code = recData[0];
  code = (code<<16) | recData[1];
  newSample = true;
}


void enableDmaInterrupt(){
	__disable_irq();
	NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
	attachInterruptVector(IRQ_DMA_CH1,spi1ISR);
	__enable_irq();
}





void initDmaSpi(){
  // Select Pin 26 for external DMA Trigger at DRl Pulse
  PORTA_PCR14 &= ~(1);
  PORTA_PCR14 &= ~(1<<2);
  PORTA_PCR14 &= ~(0b111<<8);
  PORTA_PCR14 |= 1<<8;
  /* Bits 15-19 on the PCRx_PCRn register are responsible for the Type of Interrupt
   * 0001 : Rising Edge Interrupt (Implemented by setting Bit 17 and 19)
   */
  PORTA_PCR14 &= ~(0b1111<<16);
  PORTA_PCR14 |= 1<<16; 
  
  SPI1_RSER = 0x00;
  SPI1_RSER = SPI_RSER_EOQF_RE | SPI_RSER_TFFF_RE | SPI_RSER_TFFF_DIRS | SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS;
  tx.disable();
  rx.disable();
  Trigger.disable();
  Trigger.sourceBuffer(CLEAR_FLAG,4);
  Trigger.destination((uint32_t &) SPI1_SR);
  Trigger.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTA);
  rx.destinationBuffer(recData, 8);
  rx.source((volatile uint32_t &) SPI1_POPR);
  rx.triggerAtHardwareEvent(DMAMUX_SOURCE_SPI1_RX);
  tx.sourceBuffer(data, 8);
  tx.destination((volatile uint32_t &) SPI1_PUSHR);
  tx.triggerAtHardwareEvent(DMAMUX_SOURCE_SPI1_TX);
  tx.enable();
  rx.enable();
  Trigger.enable();


  SPI1_MCR &= ~(1);
  delay(1000);
  SPI1_TCR = 0x00;
  
  rx.interruptAtCompletion();
  enableDmaInterrupt(); 
}





#endif