#include <SPI.h>
#include "src/AD5760/AD5760.h"

#define INVERT_PIN      13
#define NON_INVERT_PIN  14
#define CS_DAC          15
#define SYNC_PIN        18
AD5760 dac(CS_DAC);

/*switch Truth-Table:
 * 00: Non-Inverted
 * 11: Shorted
 * 01: Shorted
 * 10: Inverted
 */

volatile uint32_t code;
volatile uint8_t rx3;
volatile uint8_t rx2;
volatile uint8_t rx1;
volatile uint8_t rx0;
const float ADC_V_REF = 4.096;
volatile bool newSample = false;

void setup() {
  Serial.begin(115200);
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);
  pinMode(NON_INVERT_PIN, OUTPUT);
  pinMode(INVERT_PIN, OUTPUT);
  digitalWrite(NON_INVERT_PIN, LOW);
  digitalWrite(INVERT_PIN, LOW);
  dac.begin();    // initialize the DAC pins

  SPI1.begin();

  delay(500);
  dac.reset();    // Führt enableOutput() aus.
  dac.enableOutput();   // Redundant und kann gestrichen werden
  dac.setValue(0xFA0);   // DAC Setpoint wird festgelegt.
  delay(1000);
  initAdcClock();
  initSpiFifoMode();
  enableSpiFifoModeInterrupt();   //Enables interrupt after 32-Bits of Data are in the Receive-FIFO-Buffer and reads it to clear the buffer
  enableExternalInterrupt();      //Enables Falling-Edge-Interrupt on pin 2 of the Teensy-LC
  //InvertTimer.begin(InvertCurrent, 5000000); //Call InvertCurrent function which also sends a Sync-Pulse to the ADC and a Trigger for the Multimeter HP3458
}

void invertCurrent(const bool inverted) {
  digitalWriteFast(SYNC_PIN, HIGH);
  digitalWriteFast(INVERT_PIN, inverted); 
  digitalWriteFast(NON_INVERT_PIN, inverted);
  digitalWriteFast(SYNC_PIN, LOW);
}

void loop() {
  while (Serial.available()) {
    Serial.read();
  }

  static bool inverted = false;   // Note: Make sure that this value corresponds to the intial value of the output pins
  static uint32_t dataCounter = 0;
  if(newSample) {
    Serial.print((int32_t)code);
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

float codeToVoltage(const int32_t code, const float vref) {
  return ((float)code / 2147483647) * vref;
}

void initSpiFifoMode() {
  // All Interrupts are disabled
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  SPI1_C1 = 0x50;       // Enabling SPI module and configuring it as Master. All Interrupts configured by this register are disabled
  SPI1_C2 = 0x40;      // Initializes the SPI1_C2 register (0b01000000) to operate in 16-Bit mode and disables Interrupt and DMA requests
  SPI1_C3 = 0x11;
  /*Enables FIFOMODE
   * Transmit nearly empty buffer mark at 16-Bits(It's not important if this Bit is set or cleared
   * Receive Buffer nearly full mark at 32-Bits. This sets the RNFULL Interrupt Flag if 32-Bits of data are in the Receive-FIFO-Buffer
   * Interrupt Flags are cleared automatically depending on the FIFO-status
   * Transmit nearly empty Interrupt is disabled
   * Receive nearly full interrupt is disabled
   */
}

void enableExternalInterrupt() {
  /* Manual Version "KL26 Sub-Family Reference Manual, Rev. 3.2, October 2013"
   *  Pin Control Register (PORTx_PCRn) is found in Chapter 11.5.1 on p.199
   *  Interrupt Numbers and corresponding isr() are found at https://github.com/PaulStoffregen/cores/blob/master/teensy3/kinetis.h#L285
   */
  attachInterruptVector(IRQ_PORTCD, portcd_isr);
  __disable_irq();  // Disables all Interrupts

  //PORTD_PCR0 is Pin 2 on the Teensy-LC
  //TODO: CHeck in pinMode(2,INPUT) clears bit 0 and 2 of the PORTD_PCR0 register to make code easier to read
  PORTD_PCR0 &= ~(1);
  PORTD_PCR0 &= ~(1<<2);
  PORTD_PCR0 |= 1<<8;

  /* Bits 15-19 on the PCRx_PCRn register are responsible for the Type of Interrupt
   * 1010 : Falling Edge Interrupt (Implemented by setting Bit 17 and 19)
   */
  PORTD_PCR0 |= (1<<17); 
  PORTD_PCR0 |= (1<<19); 
  __enable_irq();   //Enables Interrupts
}

void enableSpiFifoModeInterrupt() {
  /* Manual Version "KL26 Sub-Family Reference Manual, Rev. 3.2, October 2013"
   * Memory map and register definitions on p.681
   * Interrupt Numbers and corresponding isr() are found at https://github.com/PaulStoffregen/cores/blob/master/teensy3/kinetis.h#L285
   */
  __disable_irq();
  NVIC_ENABLE_IRQ(IRQ_SPI1);
  attachInterruptVector(IRQ_SPI1, spi1_isr);
  SPI1_C3 |= 1<<1;                                  //Enables Receive-FIFO buffer nearly full at 32-Bits Interrupt
  __enable_irq();
}

// Interrupt Service Routine for PortD
void portcd_isr(void) {
  PORTD_ISFR = 0xFFFFFFFF;                          //Clear PORTD Interrupt Register By Writing ones to it. PORTx_ISFR defintion is found on p.202-203

  /* SPI Memory map and register definitions on p.681
   * SPI-Transaction is triggered by reading the status register SPI1_S and writing to the Data Registers SPI1_DH:DL
   * The SPI-bus operates in 16-Bit Mode. Therefore writes to the DataHigh (DH) and the DataLow(DL) is needed to start a transaction.
   * Since FIFO-Mode is enabled the SPI has 64-Bits which are filled by writing to the SPI1_DH:DL registers multiple times.
   */
  SPI1_S;                                           
  SPI1_DH = 0x00;
  SPI1_DL = 0x00;  
  SPI1_DH = 0x00;
  SPI1_DL = 0x00;
}

void spi1_isr(void) {
  // TODO: test if clearing Bit 3 on SPI1_C3 improves Timing issues
  // By clearing this Bit the Interrupt Flag registry SPI1_CI must be cleared manually
  // Uncomment the next line if SPI1_C3_INTCLR is set to 1
  //SPI1_CI=0xF; 
  SPI1_S;
  /* The SPI-bus operates in 16-Bit Mode. Therefore reads from the DataHigh (DH) and the DataLow(DL) is needed to clear those registers and enable the SPI-Bus
   * to start a new transaction by writing to the same registers.
   * Since FIFO-Mode is enabled the SPI has 64-Bits which are cleared by multiple reads from the SPI1_DH:DL registers.
   */
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

void TpmCntEnable(bool Enable) {
  if (Enable){
    TPM1_SC &= ~(1<<4);      // Clears Bit 4 [CMOD]
    TPM1_SC |= 1<<3;         // Sets Bit 3 to 1 [CMOD] --> [CMOD]=0b01 TPM counter increments on every TPM counter clock
   }
  else{
    TPM1_SC &= ~(1<<4);       // Clears Bit 4 [CMOD]
    TPM1_SC &= ~(1<<3);      // Clears Bit 3 [CMOD]
   }
}

void initAdcClock() {
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
   TPM1_MOD =  48*1-1;        // Setting MOD to 47 to reset counter when it reaches 47 after 48 Cycles

   /*
    * TPM1_C0SC must be initialized to Edge-aligned PWM
    * TPM1_C0V contains TPM counter value or match Value for output Modes
    * TPM1_POL Bit 0 and 1 controll polarity of channel 0 and 1 with 0=active high
    * TPM1_CONF check init later Counter Pause(0),Reload(1), Stop(1) or Start (0) on Trigger, GTBEEN(0),
    */
   TPM1_C0V = 0x1;        // If C0V == Counter match occurs an Channel Interrupt Flag. Minimum High-Time for the LTC2508 MCLK is 20 ns
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

   //Select Alt3 for Pin 16 to get TPM1_CH0 p.177
   PORTB_PCR0 &= ~(1<<10);  //Clear Bit 10 
   PORTB_PCR0 |= 1<<9;  //Sets Bit 9 to 1
   PORTB_PCR0 |= 1<<8;  // Sets Bit 8 to 1
}
