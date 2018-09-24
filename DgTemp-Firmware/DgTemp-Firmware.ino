#include <SPI.h>
#include "src/AD5760/AD5760.h"
#include "src/DgTemp/DgTemp.h"
#include "src/TeensyDAC/TeensyDAC.h"
#include "src/PID/src/pid.h"
#include <DMAChannel.h>


#define CS_DAC              8
#define SETPOINT            285000000
#define KP                  500
#define KI                  0
#define KD                  0
#define QN                  20
AD5760 dac(CS_DAC);
PID pid(SETPOINT, KP, KI, KD, QN, feedbackPositive);
TeensyDAC DCDC;

/*switch Truth-Table:
 * 00: Non-Inverted
 * 11: Shorted
 * 01: Shorted
 * 10: Inverted
 */

const float ADC_V_REF = 4.096;

void setup(){
  clockInit();
  DCDC.dacSetup();
  DCDC.disableDCDC();
  
  pinMode(NON_INVERT_PIN, OUTPUT);
  pinMode(INVERT_PIN, OUTPUT);
  digitalWrite(NON_INVERT_PIN, LOW);
  digitalWrite(INVERT_PIN, LOW);
  
  dac.begin();                              // initialize the DAC_CS pin
  SPI.begin();
  delay(500);                               // Deleay needed between setting DAC_CS-Pin and the first SPI-Transaction (TODO: test for minimum duration)
  dac.reset();                              // executes enableOutput
  dac.setValue(39999);                      // DAC Setpoint is set
  
  spiInit();
  timerInit();

  Serial.begin(115200);
  Serial.println("Setup Worked");

  pid.setOutputMin(250);
  pid.setOutputMax(1370);
  DCDC.enableDCDC();
}

void loop(){  
  // Flush serial buffer to submit data to Linux systems
  while (Serial.available()) {
    Serial.read();
  }

  if(sampleReceived()) {
    DCDC.setOutput(pid.compute(getCode()));
    //Serial.print(codeToVoltage(getCode(),ADC_V_REF),10);
    Serial.print((int32_t)getCode());
    Serial.print(",");
    Serial.print(GPIOD_PDOR>>7 & 1U);
    Serial.print(",");
    Serial.print(FTM0_C0V);     //Print Channel Value at rising flank of the DRL pulse
    Serial.print(",");
    Serial.print(SPI1_TCR>>16);
    Serial.print("\n");
    
    waitForSample();
  }
}

double codeToVoltage(const int32_t code, const float vref) {
  return ((double)code / 2147483647) * vref;
}
