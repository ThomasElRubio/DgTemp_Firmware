#include <SPI.h>
#include "src/AD5760/AD5760.h"

#define INVERT_PIN      4
#define NON_INVERT_PIN  5
#define SYNC_PIN        7
#define CS_DAC          8
AD5760 dac(CS_DAC);

/*switch Truth-Table:
 * 00: Non-Inverted
 * 11: Shorted
 * 01: Shorted
 * 10: Inverted
 */

volatile uint32_t code;
volatile uint16_t rx1;
volatile uint16_t rx0;
const float ADC_V_REF = 4.096;
volatile bool newSample = false;

void setup(){
  delay(100);
  SIM_CLKDIV1 |= 1<<24;

  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);
  pinMode(NON_INVERT_PIN, OUTPUT);
  pinMode(INVERT_PIN, OUTPUT);
  digitalWrite(NON_INVERT_PIN, LOW);
  digitalWrite(INVERT_PIN, LOW);

  
  dac.begin();    // initialize the DAC pin
  SPI.begin();
  delay(500);
  dac.reset();    // Führt enableOutput() aus.
  dac.enableOutput();   // Redundant und kann gestrichen werden
  dac.setValue(39999);   // DAC Setpoint wird festgelegt.
  
  delay(2000);
  initAdcClock(); 
  initSpiFifoMode();
  enableSpiInterrupt();
  Serial.begin(115200);
  Serial.println("Worked");
  enableExternalInterrupt();
}



void loop(){  
  while (Serial.available()) {
    Serial.read();
  }

  static bool inverted = false;   // Note: Make sure that this value corresponds to the intial value of the output pins
  static uint32_t dataCounter = 0;
  if(newSample) {
    Serial.print((int32_t)code);
    //Serial.print(codeToVoltage(code,ADC_V_REF),10);
    Serial.print(",");
    Serial.print(inverted);
    //Serial.print(",");
    // Serial.print(codeToVoltage(code, ADC_V_REF));
    Serial.print("\n");
    newSample = false;
    dataCounter+=1;
  }
  if(dataCounter==5007) {
    dataCounter=0;
    inverted = !inverted;
    invertCurrent(inverted);  
  }
}



void invertCurrent(const bool inverted) {
  digitalWriteFast(SYNC_PIN, HIGH);
  digitalWriteFast(INVERT_PIN, inverted); 
  //delay(1);
  digitalWriteFast(NON_INVERT_PIN, inverted);
  digitalWriteFast(SYNC_PIN, LOW);
}



float codeToVoltage(const int32_t code, const float vref) {
  return ((float)code / 2147483647) * vref;
}




void initSpiFifoMode() {
  SPI.begin();
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  //Setting Baudrate to 500kHz
  SPI0_CTAR1 |= 1<<2;
  SPI0_CTAR1 &= ~(1<<1);
  SPI0_CTAR1 |= 1;
  SPI0_CTAR1 |= 1<<16;
  //Setting delay between Interrupt an SCK
  SPI0_CTAR1 |= 1<<18;
  SPI0_CTAR1 |= 1<<19;
  SPI0_CTAR1 |= 1<<20;
  SPI0_CTAR1 |= 1<<22;
  SPI0_CTAR1 &= ~(1<<14);
  SPI0_CTAR1 |= 1<<13;
  Serial.print(SPI0_CTAR1>>12,BIN);
} 



void enableExternalInterrupt() {
  
  __disable_irq();  // Disables all Interrupts
  attachInterruptVector(IRQ_PORTD, portd_isr);
  //PORTD_PCR0 is Pin 2 on the Teensy3.6
  //TODO: CHeck in pinMode(2,INPUT) clears bit 0 and 2 of the PORTD_PCR0 register to make code easier to read
  PORTD_PCR0 &= ~(1);
  PORTD_PCR0 &= ~(1<<2);
  PORTD_PCR0 |= 1<<8;

  /* Bits 15-19 on the PCRx_PCRn register are responsible for the Type of Interrupt
   * 1010 : Falling Edge Interrupt (Implemented by setting Bit 17 and 19)
   */
  PORTD_PCR0 |= (1<<16); 
  PORTD_PCR0 |= (1<<19); 
  __enable_irq();   //Enables Interrupts
}





void enableSpiInterrupt() {
  /* 
   * Interrupt Numbers are found at https://github.com/PaulStoffregen/cores/blob/master/teensy3/kinetis.h#L285
   */
  __disable_irq();
  NVIC_ENABLE_IRQ(IRQ_SPI0);                            //Enables the IRQ_SPI0 vector
  attachInterruptVector(IRQ_SPI0, spi0_isr);            // Connects interrupt vector with a service routine
  SPI0_RSER |= 1<<28;                                   //Enables End of Queue Interrupt
  __enable_irq();
}





void portd_isr(void) {
  /*
   * The SPI0_PUSHR register is splitted in the command register (16 MSB) and data to be transmitted (16LSB)
   * every write to this register is pushed one TX FIFO register(maximum of 4 writes)
   * A queue of two 16-Bit word tranaction is added two the TX FIFO and this will start the Transaction.
   * Timing of this transaction relating to the Busy state of the ADC is not tested yet and must be looked at.
   * Therefore modifacations to both SPI0_PUSHR (16MSB) and SPI0_CTAR1 registers is needed to optimize this transaction.
   */
  PORTD_PCR0 |= 1<<24;
  SPI0_PUSHR = 0b00010000000000000000000000000000;    //Setting Bit 28 chooses 16-Bit SPI Mode
  SPI0_PUSHR = 0b00011000000000000000000000000000;    // Setting Bit 27 marks the end of a queue and sets the End-of_Queue Interrupt flag in SPI0_SR                                // Clearing ISF by writing a one to it
}


void spi0_isr(void) {
   SPI0_SR |= 1<<28;         //Writing a 1 to this Bit clears the End of Que Interrupt Flag
  /*
   * This is a dummy data read of all RX Registers
   * Certify that SPI0_POPR always points to the registers which have received the latest data
   */
  rx1 = SPI0_POPR;
  rx0 = SPI0_POPR;
  code = rx1;
  code = (code<<16) | rx0;
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
  
  /* Manual Version "K66 Sub-Family Reference Manual, Rev. 2, May 2015"
   * TPM Clocks are listed on p. 130 in figure 6-6.
   * MCGPLLCLK goes up to 180Mhz
   * First set the fields [TPMSRC] and [PLLFLLSEL] of the SIM_SOPT2 register to chose clock (p.238)
   * [TPMSRC] = 01 and [PLLFLLSRC] = 01 chooses MCGPLLCLK which goes op to 180Mhz
   * To get 180MHz set the last 4 Bits of the SIM_CLKDIV3 register to 0
   */
   
  /* The USB module operates with the same clock as the TPM-Module
   * SIM_CLKDIV2 generates USB FS clock with: Divider output clock = Divider input clock × [ (USBFRAC+1) / (USBDIV+1) ]
   */
  SIM_SCGC4 &= ~(1<<18);      //SIM_SCGC4[USBOTG]: Disables clock at USB clock gate
  SIM_SOPT2 &= ~(1<<17);      //SIM_SOPT2[PLLFLLSEL] gate to control clock selection. Clearing this Bit changes Clock from IRC48M (48MHz) to MCGPLLCLK(180MHz)
  SIM_CLKDIV2 |= 111<<1;      //SIM_CLKDIV2[USBDIV]: Clock division is now needed to get a clock that is as close to 48MHz as possible. 
  SIM_CLKDIV2 |= 1;           //SIM_CLKDIV2[USBFRAC]:
  SIM_SCGC4 |= 1<<18;         //SIM_SCGC4[USBOTG]: Enables clock at USB clock gate


   
  TpmCntEnable(false);
  //TPM clock source select set to 01
  SIM_SOPT2 |= 1<<24;     // [TPMSRC]
  SIM_SOPT2 &= ~(1<<25);  // [TPMSRC]
  
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
   TPM1_MOD = 192*1-1;        // Setting MOD to 180 to reset counter when it reaches 180. Timer repeats after t=TPM1_MOD/180MHz  180*1-1
   

   /*
    * TPM1_C0SC must be initialized to Edge-aligned PWM
    * TPM1_C0V contains TPM counter value or match Value for output Modes
    * TPM1_POL Bit 0 and 1 controll polarity of channel 0 and 1 with 0=active high
    * TPM1_CONF check init later Counter Pause(0),Reload(1), Stop(1) or Start (0) on Trigger, GTBEEN(0),
    */
   TPM1_C0V = 4;        // If C0V == Counter match occurs an Channel Interrupt Flag. An increase of one increases the hightime by 5,55ns   4
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
   
   PORTA_PCR12 |= 111<<8; // Sets Pin 3 to Output TPM1_CH0 signal
   SIM_SOPT2 &= ~(1<<17);
   SIM_CLKDIV2 |= 111<<1;
   TpmCntEnable(true);
   
}






  




