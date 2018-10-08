#include <SPI.h>
#include "src/AD5760/AD5760.h"
#include "src/AD5680/AD5680.h"
#include "src/DgTemp/DgTemp.h"
#include "src/TeensyDAC/TeensyDAC.h"
#include "src/pidlib/src/pid.h"
#include <DMAChannel.h>

//3650  1710
#define CS_AD5760              8
#define CS_AD5680              7
#define SETPOINT            300000000
static double const KU                 = 0.00091125;      //0.0005*2*5*0.5*0.5*0.9*0.9*0.9;
static double const TU                 = 65.0;
static double const SAMPLING_FREQUENCY = 61.0;
static double const KP                 = 0.000379 * 64;
static double const KI                 = 0.00000005976143046459762 * 64;     //KP * 2.0 / TU / SAMPLING_FREQUENCY;
static double const KD                 = 0.0003782486590412 * 64;     //KP * TU * SAMPLING_FREQUENCY / 3.0;
#define QN                  20
AD5760 dac(CS_AD5760);
AD5680 pidDac(CS_AD5680);
PID pid(SETPOINT, KP, KI, KD, QN, feedbackPositive);
TeensyDAC DCDC;
DgTemp dgTemp;

/*switch Truth-Table:
 * 00: Non-Inverted
 * 11: Shorted
 * 01: Shorted
 * 10: Inverted
 */

const float ADC_V_REF = 4.096;

void setup(){
  dgTemp.clockInit();
  DCDC.dacSetup();
  DCDC.disableDCDC();
  
  pidDac.begin();
  delay(10);
  pidDac.setValue(0x1FFFF);
  
  pid.setOutputMin(0);
  pid.setOutputMax(0x3FFFF);
  pid.updateOutput(0x1FFFF);
  
  
  pinMode(NON_INVERT_PIN, OUTPUT);
  pinMode(INVERT_PIN, OUTPUT);
  digitalWrite(NON_INVERT_PIN, LOW);
  digitalWrite(INVERT_PIN, LOW);
  
  dac.begin();  // initialize the DAC_CS pin
  delay(10);
  SPI.begin();                              // Deleay needed between setting DAC_CS-Pin and the first SPI-Transaction (TODO: test for minimum duration)
  dac.reset();                              // executes enableOutput
  dac.setValue(39999);                      // DAC Setpoint is set
  
  dgTemp.spiInit();
  dgTemp.timerInit();
  DCDC.enableDCDC();
  Serial.begin(115200);
  Serial.println("Setup Worked");
  
  
}

void loop(){  
  // Flush serial buffer to submit data to Linux systems
  while (Serial.available()) {
    Serial.read();
  }

  if(dgTemp.receivedSample()) {
    static bool initPid = true;
    if(initPid){
      pid.init(dgTemp.getCode());
      initPid= false;
    }
    uint32_t inputValue = dgTemp.getCode();
    uint32_t outputValue = pid.compute(inputValue);
    pidDac.setValue(outputValue);
    //Serial.println(codeToVoltage(DgT.getCode(),ADC_V_REF),10);
    // FTM0_C0V: Print Channel Value at rising flank of the DRL pulse
    Serial.printf("%u,%u,%u,%u,%u\n", inputValue, outputValue, GPIOD_PDOR>>7 & 1U, FTM0_C0V, SPI1_TCR>>16);
    dgTemp.waitForSample();
  }
}
 
double codeToVoltage(const int32_t code, const float vref) {
  return ((double)code / 2147483647) * vref;
}
