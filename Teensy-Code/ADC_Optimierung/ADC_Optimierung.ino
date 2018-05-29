#include <i2c_t3.h>
#include <Demo2222AB.h>
#include <SPI.h>
#include <AD5760.h>





#define Sync        6



uint32_t code;
float ADC_Vref = 5.0;





void setup(){
  
  Serial.begin(115200);
  initAdcClock();         
  PORTA_PCR12 |= 111<<8;  // Sets Pin 3 to Output TPM1_CH0 signal
  pinMode(Sync,OUTPUT);
  
  
  
  output_low(Sync);
  //sneaker_port_init(Sync,CONFIG_DF_16384);
  //SPI.usingInterrupt(2);
  //attachInterrupt(digitalPinToInterrupt(2), Interrupt, FALLING);
  //attachInterrupt(digitalPinToInterrupt(8), Pulse, FALLING);
  Serial.println("Ready");
 
}



void loop(){  

}

void TpmCntEnable(bool Enable){
  if (Enable){
    TPM1_SC &= ~(1<<4);      // Clears Bit 4 [CMOD]
    TPM1_SC |= 1<<3;         // Sets Bit 3 to 1 [CMOD] --> [CMOD]=0b01 TPM counter increments on every TPM counter clock
   }
  else{
    TPM1_SC &= ~(1<<4);       // Clears Bit 4 [CMOD]
    TPM1_SC &= ~(1<<3);      // Clears Bit 3 [CMOD]
   }
}

void initAdcClock(){
  
  /* Manual Version "K66 Sub-Family Reference Manual, Rev. 2, May 2015"
   * TPM Clocks are listed on p. 130 in figure 6-6.
   * MCGPLLCLK goes up to 180Mhz
   * First set the fields [TPMSRC] and [PLLFLLSEL] of the SIM_SOPT2 register to chose clock (p.238)
   * [TPMSRC] = 01 and [PLLFLLSRC] = 01 chooses MCGPLLCLK which goes op to 180Mhz
   * To get 180MHz set the last 4 Bits of the SIM_CLKDIV3 register to 0
   */
  TpmCntEnable(false);
  SIM_SOPT2 |= 1<<24;     // Sets Bit 24 to 1 [TPMSRC]
  SIM_SOPT2 &= ~(1<<25);  // Sets Bit 25 to 0 [TPMSRC]
  SIM_SOPT2 |= 1<<16;     // Sets Bit 16 to 1 [PLLFLLSRC]
  SIM_SOPT2 &= ~(1<<17);  // Sets Bit 17 to 0 [PLLFLLSRC]
  SIM_CLKDIV3 &= 0xFFFFFFF0; // Sets Bit 0-3 to a 0 to get 180MHZ with MCGPLLCLK
  /*
   * Setup of the TPM1-Instance
   * First Setup TPM1_SC (Status and Controll) register (p.1066)
   * Write anything to TPM1_CNT clears the counter
   * Set TPM1_MOD to 180 to enable TOI after 180 clocks --> TOF appears with 1 MHz
   */
   TPM1_SC |= 1<<8;         // Enables DMA for Timer Overflow Interrupt
   TPM1_SC |= 1<<6;         // Enables Timer Overflow Interrupt
   TPM1_SC &= ~(1<<5);      // TPM counter operates in up counting mode
   TPM1_SC &= ~(111);       // Clears Bit 0-2 and therefor sets the Prescale Factor to 1
   TPM1_CNT = 0;            // Resetting the Counter Register to clear Counter
   TPM1_MOD =  0xB4;        // Setting MOD to 180 to reset counter when it reaches 180
   

   /*
    * TPM1_C0SC must be initialized to Edge-aligned PWM
    * TPM1_C0V contains TPM counter value or match Value for output Modes
    * TPM1_POL Bit 0 and 1 controll polarity of channel 0 and 1 with 0=active high
    * TPM1_CONF check init later Counter Pause(0),Reload(1), Stop(1) or Start (0) on Trigger, GTBEEN(0),
    */
   TPM1_C0V = 0x4;        // If C0V == Counter match occurs an Channel Interrupt Flag.
   TPM1_C0SC |= 1<<6;     // Enables Channel Interrupt
   TPM1_C0SC |= 1<<5;     //MSB set to 1
   TPM1_C0SC &= ~(1<<4);  //MSA set to 0  
   TPM1_C0SC |= 1<<3;     //ELSB set to 1
   TPM1_C0SC &= ~(1<<2);  //ELSB set to 0 --> MSB:MSA=1:0 and ELSB:ELSA = 1:0 enables Edge aligned PWM
   TPM1_C0SC |= 1;        // Enables DMA for Chanel Interrupt

   TPM1_POL &= ~(1);      //Sets Chanel 0 Polarity to active High

   TPM1_CONF &= ~(1<<9);  //Sets GTBEEN to 0 an enables internally generated TPM counter as timebase
   TPM1_CONF &= ~(1<<16); // TPM counter start immediately once enabled
   TPM1_CONF &= ~(1<<17); // Counter continues incrementing after overflow
   
   TpmCntEnable(true);
   
   
}






  




