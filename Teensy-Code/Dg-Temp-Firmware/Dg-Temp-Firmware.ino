#include <i2c_t3.h>
#include <Demo2222AB.h>
#include <SPI.h>
#include <AD5760.h>




#define MCLK        4
#define Sync        6
#define CS_DAC      15
#define invert      16
#define buttonPin   5
#define TRIGGER     3
AD5760 DAC(CS_DAC);




uint32_t code;
uint16_t outputVoltage = 0x40;
uint32_t maximumOutput = 0xFFFF;
float Vref = 5.0;

float Meas_Voltage;




void setup(){
  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 100000);
  Serial.begin(115200);
  SPI.begin();
  pinMode(buttonPin,INPUT);
  pinMode(invert,OUTPUT);
  pinMode(MCLK,OUTPUT);
  pinMode(Sync,OUTPUT);
  pinMode(TRIGGER,OUTPUT);
  output_high(invert);
  output_low(MCLK);
  output_low(Sync);
  output_low(TRIGGER);
  DAC.begin();
  DAC.reset();
  DAC.enableOutput();
  DAC.setValue(outputVoltage);
  sneaker_port_init(Sync,CONFIG_DF_16384);
  SPI.usingInterrupt(2);
  attachInterrupt(digitalPinToInterrupt(2), Interrupt, FALLING);
  Serial.println("Ready");
 
}

void loop(){
  
  send_pulses(MCLK, Sync, 16384);
  
}





void Interrupt(){
  code= Read_32_Bits();
  Meas_Voltage = Code_to_Voltage(code,Vref);
  Serial.println(code);
  Serial.print(" , ");
  Serial.println(Meas_Voltage, 10);
}


  




