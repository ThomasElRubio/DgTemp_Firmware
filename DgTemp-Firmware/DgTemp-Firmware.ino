#include <SPI.h>
#include "src/AD5760/AD5760.h"
#include "src/DgTemp/DgTemp.h"
#include "src/TeensyDAC/TeensyDAC.h"
#include "src/pidlib/src/pid.h"
#include <DMAChannel.h>

//3650  1710
#define CS_DAC              8
#define SETPOINT            252000000
static double const  KP= 0.0005*2*5*0.5*0.5*0.9*0.9*0.9;
#define TU                  65
#define SAMPLING_FREQUENCY  61
//#define KP                  KU 
//#define KI                  0.0d //KP *2 / TU / SAMPLING_FREQUENCY
//#define KD                  0.0d //KP * TU * 10 * SAMPLING_FREQUENCY / 3 
#define QN                  20
AD5760 dac(CS_DAC);
PID pid(SETPOINT, KP, 0, 0, QN, feedbackPositive);
TeensyDAC DCDC;
DgTemp DgT;

/*switch Truth-Table:
 * 00: Non-Inverted
 * 11: Shorted
 * 01: Shorted
 * 10: Inverted
 */

const float ADC_V_REF = 4.096;

void setup(){
  DgT.clockInit();
  DCDC.dacSetup();
  DCDC.disableDCDC();
  pid.setOutputMin(0);
  pid.setOutputMax(4095);
  //pid.updateOutput(2048);
  
  
  pinMode(NON_INVERT_PIN, OUTPUT);
  pinMode(INVERT_PIN, OUTPUT);
  digitalWrite(NON_INVERT_PIN, LOW);
  digitalWrite(INVERT_PIN, LOW);
  
  dac.begin();                              // initialize the DAC_CS pin
  SPI.begin();                              // Deleay needed between setting DAC_CS-Pin and the first SPI-Transaction (TODO: test for minimum duration)
  dac.reset();                              // executes enableOutput
  dac.setValue(39999);                      // DAC Setpoint is set
  
  DgT.spiInit();
  DgT.timerInit();
  DCDC.enableDCDC();
  Serial.begin(115200);
  Serial.println("Setup Worked");
  
  
}

void loop(){  
  // Flush serial buffer to submit data to Linux systems
  while (Serial.available()) {
    Serial.read();
  }

  if(DgT.receivedSample()) {
    static bool initPid = true;
    if(initPid){
      pid.init(DgT.getCode());
      initPid= false;
    }
    DCDC.setOutput(pid.compute(DgT.getCode()));
    //Serial.println(codeToVoltage(DgT.getCode(),ADC_V_REF),10);
    Serial.print(DgT.getCode());
    Serial.print(",");
    Serial.print(DCDC.dacOutput());
    Serial.print(",");
    Serial.print(GPIOD_PDOR>>7 & 1U);
    Serial.print(",");
    Serial.print(FTM0_C0V);     //Print Channel Value at rising flank of the DRL pulse
    Serial.print(",");
    Serial.print(SPI1_TCR>>16);
    Serial.print("\n");
    DgT.waitForSample();
  }
  
}
 
double codeToVoltage(const int32_t code, const float vref) {
  return ((double)code / 2147483647) * vref;
}
