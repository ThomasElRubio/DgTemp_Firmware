#ifndef DGTEMP_CPP
#define DGTEMP_CPP


#include "DgTemp.h"
#include <Arduino.h>
#include <SPI.h>
#include <DMAChannel.h>






volatile uint32_t DgTemp::recData[] = {0, 0};
const uint32_t DgTemp::CLEAR_FLAGS[] = {0xFF0F0000};
const uint32_t DgTemp::data[] = {SPI_RESUME_TRANSACTION, SPI_END_TRANSACTION};
volatile bool DgTemp::newSample = false;
volatile uint32_t DgTemp::code = 0;
DMAChannel DgTemp::tx = DMAChannel();
DMAChannel DgTemp::rx = DMAChannel();
DMAChannel DgTemp::Trigger = DMAChannel();


// Interrupt-Service-Routine that saves the received data from the ADC
void DgTemp::spi1ISR(void) {
  rx.clearInterrupt();
  code = recData[0];
  code = (code<<16) | recData[1];
  newSample = true;
}



// Interrupt-Service-Routine that controls the inversion of the current
void DgTemp::ftm_ISR(void){
	FTM0_STATUS;                                       					 				// To clear FTM channel interrupt the FTM0_Satus register must be read and cleared by writing a 0 to it afterwards
	FTM0_STATUS = 0;                                    									// Writing a 0 to the FTM0_STATUS register clears all interrupts on all channels
	
	// Inverting of Current
	//GPIOA_PTOR |= (!(SPI1_TCR % (2*(NUMBER_OF_SAMPLES + SAMPLES_UNTIL_SETTLES)<<16)))<<13;			// This line toogles the output of the PIN 4 after (NUMBER_OF_SAMPLES + SAMPLES_UNTIL_SETTLES) are made
	//GPIOD_PTOR |= (!(SPI1_TCR % (2*(NUMBER_OF_SAMPLES + SAMPLES_UNTIL_SETTLES)<<16)))<<7;			// This line toogles the output of the PIN 5 after (NUMBER_OF_SAMPLES + SAMPLES_UNTIL_SETTLES) are made
}


void DgTemp::clockInit(){
	this->moduleClockGateEnable();
	this->initClocks();
	this->moduleClockGateEnable();
}


void DgTemp::timerInit(){
	this->initFlexTimer();
	this->initAdcClock();
	this->timerCounterEnable(true);
}


void DgTemp::spiInit(){
	this->initSpiBus();
	this->initDmaSpi();
}




void DgTemp::initDmaSpi(){
	// Select Pin 26 for external DMA Trigger at DRL Pulse
	PORTA_PCR14 &= ~(1);															
	PORTA_PCR14 &= ~(1<<2);
	PORTA_PCR14 &= ~(0b111<<8);
	PORTA_PCR14 |= 1<<8;
	/*	Bits 16-19 on the PCRx_PCRn register are responsible for the Type of Interrupt
		0b0001 : Rising Edge DMA request (Implemented by setting Bit 16)*/
	PORTA_PCR14 &= ~(0b1111<<16);				// PORTA_PCR14[IRQC]: Clearing all bits in this part of the registry	
	PORTA_PCR14 |= 1<<16; 						// PORTA_PCR14[IRQC]: Setting bit 16 enables DMA request at a rising edge
	
	SPI1_RSER = 0x00;							//Clearing all bits disable all interrupts of the SPI1 bus 
	
	/*	Enable all needed Interrupts:
		SPI_RSER_EOQF_RE: EndOfQueue signals the end of a SPI-transaction
		SPI_RSER_TFFF_RE: Enables request to Fill Transmit FIFO when it is empty 
		SPI_RSER_TFFF_DIRS: Selects DMA request for the Fill Transmit FIFO event
		SPI_RSER_RFDF_RE: Enables request to Drain Receive FIFO when it is full
		SPI_RSER_RFDF_DIRS: Selects DMA request for the Drain Receive FIFO event
	*/
	SPI1_RSER = SPI_RSER_EOQF_RE | SPI_RSER_TFFF_RE | SPI_RSER_TFFF_DIRS | SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS;	
	/*
		Setup of the DMAChannels
		tx: This Channel fills the SPI1_PUSHR register which contains the configuration of the spi-transaction.
		rx: This Channel reads the received data out of the spi receive buffer and saves it in an array
		Trigger: This Channel is triggered when a rising edge of the DRL-Pulse is send from the ADC which clears all flags on the SPI1-Bus which enables the spi-transaction
	*/
	tx.disable();																// Disable DMAChannel during setup
	rx.disable();																// Disable DMAChannel during setup
	Trigger.disable();															// Disable DMAChannel during setup
	Trigger.sourceBuffer(CLEAR_FLAGS,4);											// Setting data source 
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
  SPI1_TCR = 0x00;
  
  rx.interruptAtCompletion();
  enableDmaInterrupt(); 
}




void DgTemp::moduleClockGateEnable(){
	SIM_SCGC2 |= SIM_SCGC2_TPM2 | SIM_SCGC2_TPM1;       					// Enable TPM2 Module and TPM1 Module
	SIM_SCGC6 |= SIM_SCGC6_FTM0 | SIM_SCGC6_SPI0 | SIM_SCGC6_SPI1; 			// Enable FlexTimer, SPI0 and SPI1
}


void DgTemp::initClocks(){
	//System Clock Divider Register 1 controls the prescalers of the Busclock, Core Clock etc.
	SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(0b0011);                     	// Setting Bus Clock divider from 3 to 4 to get a 48MHz Bus-Clock (0b0011)
	
	//Stop all Counter by deselecting the clock
	TPM1_SC &= (0b11<<3);                                       	// TPM1_SC[CMOD]: Clearing those bits(0b00) disables TPM1 counter 
	TPM2_SC &= (0b11<<3);                                       	// TPM1_SC[CMOD]: Clearing those bits(0b00) disables TPM2 counter
	
	// Clear write Protection on FTM0 registers
	if ((FTM0_FMS >> 6) & 1U) FTM0_MODE |= 1<<2;
	FTM0_SC &= ~(0b11<<3);                                 		// FTM0_SC[CLKS] : Clearing those bits (ob00) disables FTM0 counter
	
	// Reset all Counter
	FTM0_CNTIN = 0x00;                                           	// Setting initial value of the counter to 0
	FTM0_CNT = 0x00;                                             	// Writing any value to this counter resets the counter to its CNTIN value
	TPM1_CNT = 0x00;                                            	// Writing any value to this register resets the counter to zero
	TPM2_CNT = 0x00;                                            	// Writing any value to this register resets the counter to zero
	
	// Configuring all clocks that are needed for the counter modules to work
	SIM_SCGC4 &= ~(SIM_SCGC4_USBOTG);                            	// Disables clock at USB clock gate so the usb clock can be changed
	SIM_SOPT2 &= ~(SIM_SOPT2_IRC48SEL); 								// Clear PLL/FLL clock select bits 
	SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL;                             	// SIM_SOPT2[PLLFLLSEL] gate to control clock selection. Clearing Bit 17 and setting bit 16 changes Clock from IRC48M (48MHz) to MCGPLLCLK(192MHz)
	SIM_CLKDIV2 |= 0b111<<1;                                       // SIM_CLKDIV2[USBDIV]: Setting USBDIV=7. Divider input clock=192MHz
	SIM_CLKDIV2 |= 1;                                             // SIM_CLKDIV2[USBFRAC]: Setting USBFRAC=1. Divider output clock = Divider input clock × [ (USBFRAC+1) / (USBDIV+1) ]
	SIM_SCGC4 |= SIM_SCGC4_USBOTG;                                	// SIM_SCGC4[USBOTG]: Enables clock at USB clock gate
	
	
	//TPM clock source select set to 0b01 to select MCGPLLCLK
	SIM_SOPT2 |= 1<<24;                                      		// SIM_SOPT2[TPMSRC]: Setting bit 24 and clearing bit 25 selects PLLFLLCLK as the TPM clock
	SIM_SOPT2 &= ~(1<<25);                                       	// SIM_SOPT2[TPMSRC]
	//Setting prescaler of the FTM module counter
	FTM0_SC &= ~(1);      										// FTM0_SC[PS]: Set FTM0_SC[PS] to 0b100 to get a prescaler of 16 which reduces the counter clock down to 3MHz
	FTM0_SC &= ~(1<<1);											// FTM0_SC[PS]
	FTM0_SC |= 1<<2;												// FTM0_SC[PS]
}


void DgTemp::initFlexTimer(){
	// Setting counter to up-counting mode
	FTM0_SC &= ~(1<<5);	 										// FTM0_SC[CPWMS]: Counter is in Up-Counting Mode. This is also needed to use Input-Capture-Mode
	
 
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
	attachInterruptVector(IRQ_FTM0, ftm_ISR);
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


void DgTemp::initAdcClock(){  
	// Manual Version "K66 Sub-Family Reference Manual, Rev. 2, May 2015"

  
	/* Setup of the TPM1-Instance
	* First Setup TPM1_SC (Status and Controll) register (p.1066)
	* Write anything to TPM1_CNT clears the counter
	* Set TPM1_MOD to 180 to enable TOI after 192 clocks --> TOF appears with 1 MHz*/
	TPM1_SC &= ~(1<<5);      	// TPM counter operates in up counting mode
	TPM1_SC &= ~(0b111);      // Clears Bit 0-2 and therefor sets the Prescale Factor to 1
	TPM1_CNT = 0;           	// Resetting the Counter Register to clear Counter
	TPM1_MOD = 192*1-1;      	// Setting MOD to 180 to reset counter when it reaches 180. Timer repeats after t=TPM1_MOD/180MHz  180*1-1
   
	/* TPM1_C0SC must be initialized to Edge-aligned PWM
	 * TPM1_C0V contains TPM counter value or match Value for output Modes
	 * TPM1_POL Bit 0 and 1 controll polarity of channel 0 and 1 with 0=active high
	 * TPM1_CONF check init later Counter Pause(0),Reload(1), Stop(1) or Start (0) on Trigger, GTBEEN(0),
	*/
	TPM1_C0V = 4;        		// If C0V == Counter match occurs an Channel Interrupt Flag. An increase of one increases the hightime by 5,55ns 
	TPM1_C0SC |= 1<<5;     	//MSB set to 1
	TPM1_C0SC &= ~(1<<4);  	//MSA set to 0  
	TPM1_C0SC |= 1<<3;     	//ELSB set to 1
	TPM1_C0SC &= ~(1<<2);  	//ELSB set to 0 --> MSB:MSA=1:0 and ELSB:ELSA = 1:0 enables Edge aligned PWM
	TPM1_POL &= ~(1);      	//Sets Chanel 0 Polarity to active High


  
	TPM1_CONF &= ~(1<<9); 		      // Sets GTBEEN to 0 an enables internally generated TPM counter as timebase
	TPM1_CONF |= 1<<16;	      	// TPM counter start immediately after Trigger
	TPM1_CONF &= ~(1<<17);      	  // Counter continues incrementing after overflow
	TPM1_CONF |= 1<<27;             // Trigger occours when FTM0_CNT=0
	PORTA_PCR12 |= 0b111<<8;      	// Sets Pin 3 to Output TPM1_CH0 signal
	
	//Configure TPM in Combine-Mode to get a continues Sync Pulse for both ADC
	TPM2_SC = 0x00;                  // count on TPM counter clock, prescaler 0
	TPM2_CNT = 0;                    //Resseting Counter
	TPM2_MOD = 192-1;                // 1MHz Clock
	TPM2_C0V = 130;                 // Rising edge at Countervalue 130
	TPM2_C1V = 150;                 // Falling Edge at Countervalue 150 
	TPM2_C0SC = 0x24;               // Configuring channel 0 for combine Mode with output on low at CounterINIT
	TPM2_COMBINE |= 1;              // Enables combination of Channol 0 and 1 of TPM2
	TPM2_CONF &= ~(1<<9);           // Sets GTBEEN to 0 an enables internally generated TPM counter as timebase
	TPM2_CONF |= 1<<16;             // TPM counter start immediately after Trigger
	TPM2_CONF |= 1<<17;             // Counter stop incrementing after Overflow
	TPM2_CONF |= 1<<18;             // Counter Resets at Trigger
	TPM2_CONF |= 1<<27;           // Trigger occours when FTM0_CNT=0
  
	//Selecting Alt 6 for PORTB_PCR0 to get Sync Pulse on Pin 16
	PORTB_PCR18 |= 1<<10;
	PORTB_PCR18 |= 1<<9;    
	PORTB_PCR18 &= ~(1<<8);
	PORTB_PCR18 |= 1<<6; 
}


void DgTemp::timerCounterEnable(bool Enable){
	if (Enable){
		FTM0_CNT = 0;
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


void DgTemp::initSpiBus(){ 	
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


bool DgTemp::receivedSample(){
	return this->newSample;
}


void DgTemp::waitForSample(){
	this->newSample = false;
}



void DgTemp::enableDmaInterrupt(){
	__disable_irq();
	NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
	attachInterruptVector(IRQ_DMA_CH1,spi1ISR);
	__enable_irq();
}






uint32_t DgTemp::getCode(){
	return this->code;
}













#endif