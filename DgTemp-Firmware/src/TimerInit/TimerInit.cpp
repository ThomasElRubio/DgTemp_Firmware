

#include "TimerInit.h"


void TimerInit::moduleClockGateEnable(){
	SIM_SCGC2 |= SIM_SCGC2_TPM2 | SIM_SCGC2_TPM1;       // Enable TPM2 Module
	SIM_SCGC6 |= SIM_SCGC6_FTM0 | SIM_SCGC6_SPI0 | SIM_SCGC6_SPI1; //| SIM_SCGC6_DMAMUX;  //Enable FlexTimer,SPI0 and DMAMUX
}