#ifndef __ARDUINO_TimerInit
#define __ARDUINO_TimerInit


#include <Arduino.h>
#include <SPI.h>
#include <DMAChannel.h>



class TimerInit {
	public:
		void start();
		volatile uint32_t recData[] = {0, 0};
		
		
	protected:
		void moduleClockGateEnable();
		void initCLocks();
		void initFlexTimer();
		void initAdcClock();
		void TpmCntEnable(bool enabled);
		void initSpiMode();
		void initDmaSpi();
		const uint32_t CLEAR_FLAG[] = {0xFF0F0000};
		const uint32_t data[]={0b00010000000000000000000000000000,0b00011000000000000000000000000000};
		
	
	
	
	
	
};






#endif