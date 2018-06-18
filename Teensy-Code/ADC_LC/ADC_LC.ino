#include <i2c_t3.h>
#include <Demo2222AB.h>
#include <SPI.h>
#include <AD5760.h>






#define Sync        6


volatile uint32_t code;
volatile uint8_t rx3;
volatile uint8_t rx2;
volatile uint8_t rx1;
volatile uint8_t rx0;
float ADC_Vref = 5.0;
float Meas_Voltage;
bool newSample = false;


void setup(){
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 100000);
  Serial.begin(115200);
  pinMode(Sync,OUTPUT);
  output_low(Sync);
  
  initSpiFifoMode();
    
  sneaker_port_init(Sync,CONFIG_DF_16384);    //Initialisiert den FPGA und damit den Downsampling-Faktor des ADC auf dem Demoboard 2222 AB

  initAdcClock();     
  enableExternalInterrupt();  //Enables Falling Edge Innterrupt on Pin 2 of the Teensy-LC
}

void loop(){
  if(newSample){ 
  Serial.println(Code_to_Voltage(code, ADC_Vref),10);
  newSample = false;
  }
}


void initSpiFifoMode(){
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(500000,MSBFIRST, SPI_MODE0));
  SPI1_C1 &= ~(1<<7); // Disable SPI Interrupts for SPRF, MODF and receive buffer full on FIFOMODE
  SPI1_C1 = ~(1<<5);  // DIsables Interrupt when Transmit buffer is empty
  SPI1_C2 |= 1<<6;    //Enables 16-Bit Mode on the SPI-Bus
  /*Enables FIFOMODE
   * Transmit nearly empty buffer mark at 32-Bits
   * Receive Buffer nearly full mark at 32-Bits. This sets the RNFULL Interrupt Flag 
   * Interrupt Flags are cleared automatically depending on the FIFO-status
   * Transmit nearly empty Interrupt is disabled
   * Receive nearly full interrupt is enabled and this interrupt is used to trigger reading the data afet SPI transaction
   */
  SPI1_C3 = 0x33;     //Enables FIFOMODE T-FIFO nearly empty at 32 bits
}


void enableExternalInterrupt(){
  attachInterruptVector(IRQ_PORTCD, portcd_isr);
  
  __disable_irq();
  
  //PORTD_PCR0 is Pin 2 on the Teensy-LC
  //TODO: CHeck in pinMode(2,INPUT) clears bit 0 and 2 of the PORTD_PCR0 register to make code easier to read
  PORTD_PCR0 &= ~(1);
  PORTD_PCR0 &= ~(1<<2);
  PORTD_PCR0 |= 1<<8;
  
  /*Bits 15-19 on the PCRx_PCRn register are responsible for the Type of Interrupt
   * 1010 : Falling Edge (Implemented by setting Bit 17 and 19
   * 
   */
  PORTD_PCR0 |= (1<<17); 
  PORTD_PCR0 |= (1<<19); 
  __enable_irq();
}

void enableSpiFifoModeInterrupt(){
  attachInterruptVector(IRQ_SPI1, spi1_isr);
}




// Interrupt Service Routine for PortD
void portcd_isr(void){
  PORTD_ISFR = 0xFFFFFFFF;    //Clear PORTD Interrupt Register By Writing ones to it
  SPI1_S;             
  SPI1_DH = 0x00;
  SPI1_DL = 0x00;  
  SPI1_DH = 0x00;
  SPI1_DL = 0x00;
}


void spi1_isr(void){
  // TODO: test if clearing Bit 3 on SPI1_C3 improves Timing issues
  //  By clearing this Bit the Interrupt Flag registry SPI1_CI must be cleared manually
  rx3 = SPI1_DH;
  rx2 = SPI1_DL;
  rx1 = SPI1_DH;
  rx0 = SPI1_DL;
  code = rx3;
  code = (code<<8) | rx2;
  code = (code<<8) | rx1;
  code = (code<<8) | rx0;
  newSample = true;
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
  
  /* Manual Version "KL26 Sub-Family Reference Manual, Rev. 3.2, October 2013"
   * TPM Clocks are listed on p. 135 in figure 5-5.
   * MCGFLLCLK goes up to 48Mhz
   * First set the fields [TPMSRC] and [PLLFLLSEL] of the SIM_SOPT2 register to chose clock (p.211)
   * [TPMSRC] = 01 and [PLLFLLSRC] = 1 chooses MCGFLLCLK which goes op to 48Mhz
   *
   */
  TpmCntEnable(false);
  SIM_SOPT2 |= 1<<24;     // Sets Bit 24 to 1 [TPMSRC]
  SIM_SOPT2 &= ~(1<<25);  // Sets Bit 25 to 0 [TPMSRC]
  SIM_SOPT2 |= 1<<16;
  //SIM_SOPT2 &= ~(1<<16);  // Sets Bit 16 to 0 [PLLFLLSRC] to get MCGFLLCLK
  /*
   * Setup of the TPM1-Instance
   * First Setup TPM1_SC (Status and Controll) register (p.567)
   * Write anything to TPM1_CNT clears the counter
   * Set TPM1_MOD to 47 to enable TOI after 48 clocks --> TOF appears with 1 MHz
   */
   TPM1_SC |= 1<<8;         // Enables DMA for Timer Overflow Interrupt
   TPM1_SC |= 1<<6;         // Enables Timer Overflow Interrupt
   TPM1_SC &= ~(1<<5);      // TPM counter operates in up counting mode
   TPM1_SC &= ~(111);       // Clears Bit 0-2 and therefor sets the Prescale Factor to 1
   TPM1_CNT = 0;            // Resetting the Counter Register to clear Counter
   TPM1_MOD =  0x2F;        // Setting MOD to 47 to reset counter when it reaches 47 after 48 Cycles
   

   /*
    * TPM1_C0SC must be initialized to Edge-aligned PWM
    * TPM1_C0V contains TPM counter value or match Value for output Modes
    * TPM1_POL Bit 0 and 1 controll polarity of channel 0 and 1 with 0=active high
    * TPM1_CONF check init later Counter Pause(0),Reload(1), Stop(1) or Start (0) on Trigger, GTBEEN(0),
    */
   TPM1_C0V = 0x3;        // If C0V == Counter match occurs an Channel Interrupt Flag.
   TPM1_C0SC |= 1<<6;     // Enables Channel Interrupt
   TPM1_C0SC |= 1<<5;     //MSB set to 1
   TPM1_C0SC &= ~(1<<4);  //MSA set to 0  
   TPM1_C0SC |= 1<<3;     //ELSB set to 1
   TPM1_C0SC &= ~(1<<2);  //ELSA set to 0 --> MSB:MSA=1:0 and ELSB:ELSA = 1:0 enables Edge aligned PWM
   TPM1_C0SC |= 1;        // Enables DMA for Chanel Interrupt


   TPM1_CONF &= ~(1<<9);  //Sets GTBEEN to 0 an enables internally generated TPM counter as timebase
   TPM1_CONF &= ~(1<<16); // TPM counter start immediately once enabled
   TPM1_CONF &= ~(1<<17); // Counter continues incrementing after overflow
   
   TpmCntEnable(true);

   //Select Alt3 for Pin 16 to get TPM1_CH0
   PORTB_PCR0 &= ~(1<<10);  //Clear Bit 10 
   PORTB_PCR0 |= 1<<9;  //Sets Bit 9 to 1
   PORTB_PCR0 |= 1<<8;  // Sets Bit 8 to 1
   
}



