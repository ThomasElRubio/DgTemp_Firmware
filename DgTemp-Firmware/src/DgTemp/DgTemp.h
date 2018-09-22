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
//static inline void initAdcClock();
void timerCounterEnable(bool Enable);
void initSpiBus();
void initDmaSpi();
void enableDmaInterrupt();


static inline void initAdcClock(){  
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




#endif