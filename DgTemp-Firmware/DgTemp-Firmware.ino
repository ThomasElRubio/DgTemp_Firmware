#include <SPI.h>
#include "src/AD5760/AD5760.h"
#include "src/DgTemp/DgTemp.h"
#include <DMAChannel.h>


#define CS_DAC          8
AD5760 dac(CS_DAC);
DgTempClass DgTemp;

/*switch Truth-Table:
 * 00: Non-Inverted
 * 11: Shorted
 * 01: Shorted
 * 10: Inverted
 */


const float ADC_V_REF = 4.096;





  


void setup(){
  DgTemp.clockInit();
  pinMode(NON_INVERT_PIN, OUTPUT);
  pinMode(INVERT_PIN, OUTPUT);
  digitalWrite(NON_INVERT_PIN, LOW);
  digitalWrite(INVERT_PIN, LOW);
  
  dac.begin();                              // initialize the DAC_CS pin
  SPI.begin();
  delay(500);                               // Deleay needed between setting DAC_CS-Pin and the first SPI-Transaction (TODO: test for minimum duration)
  dac.reset();                              // executes enableOutput
  dac.setValue(39999);                      // DAC Setpoint is set

  DgTemp.spiInit();
  DgTemp.timerInit();
  
  Serial.begin(115200);
  Serial.println("Setup Worked");



  
 
  
}



void loop(){  
  // Flush serial buffer to submit data to Linux systems
  while (Serial.available()) {
    Serial.read();
  }
  if(newSample) {
    Serial.print((int32_t)code);
    //Serial.print(codeToVoltage(code,ADC_V_REF),10);
    Serial.print(",");
    Serial.print(GPIOD_PDOR>>7 & 1U);
    Serial.print(",");
    Serial.print(FTM0_C0V);     //Print Channel Value at rising flank of the DRL pulse
    Serial.print(",");
    Serial.print(SPI1_TCR>>16);
    Serial.print("\n");
    
    newSample = false;
  }
  
}






float codeToVoltage(const int32_t code, const float vref) {
  return ((float)code / 2147483647) * vref;
}



  




