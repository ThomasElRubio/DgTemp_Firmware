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
volatile uint32_t code;
const uint32_t SPI_RESUME_TRANSACTION = 0b1 << 28; 
const uint32_t SPI_END_TRANSACTION = 0b11 << 27;
const uint32_t CLEAR_FLAG[] = {0xFF0F0000};
const uint32_t data[] = {SPI_RESUME_TRANSACTION,SPI_END_TRANSACTION};

volatile bool newSample = false;

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

void moduleClockGateEnable(){
	SIM_SCGC2 |= SIM_SCGC2_TPM2 | SIM_SCGC2_TPM1;       // Enable TPM2 Module
	SIM_SCGC6 |= SIM_SCGC6_FTM0 | SIM_SCGC6_SPI0 | SIM_SCGC6_SPI1; // Enable FlexTimer,SPI0 and DMAMUX
}

void ftmISR(void){
	FTM0_STATUS;                                        // To clear FTM channel interrupt the FTM0_Satus register must be read and cleared by writing a 0 to it afterwards
	FTM0_STATUS = 0;                                    // Writing a 0 to the FTM0_STATUS register clears all interrupts on all channels
	//GPIOA_PTOR |= (!(SPI1_TCR % (2*(NUMBER_OF_SAMPLES + SAMPLES_UNTIL_SETTLES)<<16)))<<13;
	//GPIOD_PTOR |= (!(SPI1_TCR % (2*(NUMBER_OF_SAMPLES + SAMPLES_UNTIL_SETTLES)<<16)))<<7;
}

void spi1_isr(void) {
  rx.clearInterrupt();
  code = recData[0];
  code = (code<<16) | recData[1];
  newSample = true;
}


void initClocks(){
	//System Clock Divider Register 1 controls the prescalers of the Busclock, Core Clock etc.
	SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(0b0011);                     // Setting Bus Clock divider from 3 to 4 to get a 48MHz Bus-Clock (0b0011)
	
	//Stop all Counter by deselecting the clock
	TPM1_SC &= (0b11<<3);                                           // TPM1_SC_CMOD: 00 -> Disable TPM1 counter 
	TPM2_SC &= (0b11<<3);                                           // TPM1_SC_CMOD: 00 -> Disable TPM2 counter
	
	// Clear write Protection on FTM0 registers
	if ((FTM0_FMS >> 6) & 1U) FTM0_MODE |= 1<<2;
	FTM0_SC &= ~(0b11<<3);                                          // FTM0_SC_CLKS : 00 -> Disable FTM counter
	
	// Reset all Counter
	FTM0_CNTIN = 0x00;                                               // Setting initial value of the counter to 0
	FTM0_CNT = 0x00;                                                 // Writing any value to this counter resets the counter to its CNTIN value
	TPM1_CNT = 0x00;                                                 // Writing any value to this register resets the counter to zero
	TPM2_CNT = 0x00;                                                 // Writing any value to this register resets the counter to zero
	
	// Configuring all clocks that are needed for the counter modules to work
	SIM_SCGC4 &= ~(SIM_SCGC4_USBOTG);                                // Disables clock at USB clock gate so the usb clock can be changed
	SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL;                                // SIM_SOPT2[PLLFLLSEL] gate to control clock selection. Clearing this Bit changes Clock from IRC48M (48MHz) to MCGPLLCLK(192MHz)
	SIM_CLKDIV2 |= 0b111<<1;                                         // SIM_CLKDIV2[USBDIV]: Clock division is now needed to get a clock of 48MHz. 
	SIM_CLKDIV2 |= 1;                                                // SIM_CLKDIV2[USBFRAC]:
	SIM_SCGC4 |= SIM_SCGC4_USBOTG;                                   // SIM_SCGC4[USBOTG]: Enables clock at USB clock gate
	
	
	//TPM clock source select set to 0b01 to select MCGPLLCLK
	SIM_SOPT2 |= 1<<24;                                              // [TPMSRC]
	SIM_SOPT2 &= ~(1<<25);                                           // [TPMSRC]
	FTM0_SC &= ~(1);      // Set FTM0_SC[PS] to 100 to get a prescaler of 16 which reduces the counter clock down to 3MHz
	FTM0_SC &= ~(1<<1);
	FTM0_SC |= 1<<2;
}

void initFlexTimer(){
	// Setting counter to up-counting mode
	FTM0_SC &= ~(1<<5);	 	// Counter is in Up-Counting Mode. This is also needed to use Input-Capture-Mode
	



 
	//Setting one channel to Input capture mode to check if a glitch has occured by setting MSB,MSA,ELSB and ELSA as follows
	//Channel Interrupt is not needed but can be utilized
	FTM0_COMBINE &= (1<<2);   //DECAPEN: Disabling Dual Edge Capture Mode
	FTM0_COMBINE &= ~(1);     //COMBINE: Disabling Combine feature for channels 0 and 1
	FTM0_C0SC &= ~(1<<5);     //MSB
	FTM0_C0SC &= ~(1<<4);     //MSA 
	//Rising Edge
	FTM0_C0SC &= ~(1<<3),     //ELSB
	FTM0_C0SC |= 1<<2;        //ELSA
	//Falling Edge
	FTM0_C1SC &= ~(1<<5);     //MSB
	FTM0_C1SC &= ~(1<<4);     //MSA 
	FTM0_C1SC |= 1<<3;        //ELSB
	FTM0_C1SC &= ~(1<<2);     //ELSA
	SIM_SOPT8 |= 1;
	FTM0_EXTTRIG |= 1<<6;     //Enabling Trigger at countervalue of 0 to trigger ADC-Clock module and Sync-Pulse Module
  
	//Enabling two Edge-Aligned PWM Channels to simulate a CS for two ADC
	FTM0_C2SC |= 1<<5;        //MSB:MSA set to 1X
	FTM0_C2SC &= ~(1<<3);
	FTM0_C2SC |= 1<<2;
	FTM0_C2V = 192;
	FTM0_C3SC |= 1<<5;        //MSB:MSA set to 1X
	FTM0_C3SC &= ~(1<<2);
	FTM0_C3SC |= 1<<3;
	FTM0_C3V = 192;
	
	
	__disable_irq();
	NVIC_ENABLE_IRQ(IRQ_FTM0);
	attachInterruptVector(IRQ_FTM0, ftmISR);
	FTM0_C0SC |= 1<<6;        		// Enable Channel Interrupt
	__enable_irq();
	
	//Setting Counter Initvalue
	FTM0_CNTIN = 0;         		//Setting this Register to 0 garanties a counter that always starts a 0
	FTM0_MOD = (16384*3)-1;      	//Timeroverflow occours at Countervalue=MOD which resets the counter at a frequency of 61.02Hz
	
	//Setting Pin 22 of the Teensy 3.6 to FTM0_CH0 by using MUX alternative 4
	PORTC_PCR1 |= 1<<10; 
	PORTC_PCR1 &= ~(1<<9);
	PORTC_PCR1 &= ~(1<<8); 
	//Setting Pin 23 of the Teensy 3.6 to FTM0_CH1 by using MUX alternative 4
	PORTC_PCR2 |= 1<<10; 
	PORTC_PCR2 &= ~(1<<9);
	PORTC_PCR2 &= ~(1<<8); 
	
	PORTC_PCR3 |= 1<<10;
	PORTC_PCR3 &= ~(1<<9);
	PORTC_PCR3 &= ~(1<<8);
	PORTC_PCR4 |= 1<<10;
	PORTC_PCR4 &= ~(1<<9);
	PORTC_PCR4 &= ~(1<<8);
}



void initDmaSpi(){
  // Select Pin 26 for external DMA Trigger at DRl Pulse
  PORTA_PCR14 &= ~(1);
  PORTA_PCR14 &= ~(1<<2);
  PORTA_PCR14 &= ~(0b111<<8);
  PORTA_PCR14 |= 1<<8;
  /* Bits 15-19 on the PCRx_PCRn register are responsible for the Type of Interrupt
   * 1010 : Falling Edge Interrupt (Implemented by setting Bit 17 and 19)
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

void initSpiBus(){ 	
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

void enableDmaInterrupt(){
	__disable_irq();
	NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
	attachInterruptVector(IRQ_DMA_CH1,spi1_isr);
	__enable_irq();
}


void timerCounterEnable(bool Enable){
	if (Enable){
		TPM1_SC |= 1<<3;         // Sets Bit 3 to 1 [CMOD] --> [CMOD]=0b01 TPM counter increments on every TPM counter clock
		TPM2_SC |= 1<<3;         // Sets Bit 3 to 1 [CMOD] --> [CMOD]=0b01 TPM counter increments on every TPM counter clock
		FTM0_SC |= 1<<3;         //Starting FTM Counter 
		FTM0_FMS |= 1<<6;         //Writing a 1 to WPEN also clears FTM0_MODE[WPDIS] and enables write protection
	}
	else{
		TPM1_SC &= ~(1<<4);       // Clears Bit 4 [CMOD]
		TPM1_SC &= ~(1<<3);       // Clears Bit 3 [CMOD]
		TPM2_SC &= ~(1<<4);       // Clears Bit 4 [CMOD]
		TPM2_SC &= ~(1<<3);       // Clears Bit 3 [CMOD]
		
   }
}


#endif