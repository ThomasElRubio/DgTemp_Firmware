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

// 1057 = 50µA
// 20AF = 100µA


uint32_t code;
uint16_t DAC_INIT_Voltage = 0x1057;
uint32_t maximumOutput = 0xFFFF;
int DF = 16384;
int conversion = 1;
float ADC_Vref = 5.0;
float DAC_Vref = 10.0;
float DAC_Voltage = 0;
float Meas_Voltage;
bool state =  true;




void setup(){
  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 100000);
  Serial.begin(115200);
  SPI.begin();
  pinMode(buttonPin,INPUT);
  pinMode(invert,OUTPUT);
  pinMode(MCLK,OUTPUT);
  pinMode(Sync,OUTPUT);
  pinMode(TRIGGER,OUTPUT);
  output_low(invert);
  output_low(MCLK);
  output_low(Sync);
  output_low(TRIGGER);
  DAC.begin();
  DAC.reset();
  DAC.enableOutput();
  DAC.setValue(DAC_INIT_Voltage);
  sneaker_port_init(Sync,CONFIG_DF_16384);
  SPI.usingInterrupt(2);
  attachInterrupt(digitalPinToInterrupt(2), Interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(8), Pulse, FALLING);
  Serial.println("Ready");
}

void loop(){
  //delay(17);
  /*output_high(Sync);
  delayMicroseconds(1);
  output_low(Sync);*/
  //send_pulses(MCLK,Sync ,1);
  
  /*output_high(TRIGGER);
  delayMicroseconds(10);
  output_high(invert);
  output_low(TRIGGER);
  delay(460);
  output_high(TRIGGER);
  delayMicroseconds(10);
  output_low(invert);
  output_low(TRIGGER);
  delay(460);*/
  

}


void Pulse(){
  if( conversion < DF){
    digitalWriteFast(MCLK,LOW);
    digitalWriteFast(MCLK,HIGH);
    conversion++;
  }
  else conversion = 1;
}

void Interrupt(){
  code= Read_32_Bits(MCLK);  
  Meas_Voltage = Code_to_Voltage(code,ADC_Vref);
  Serial.println(Meas_Voltage, 10);
 // Serial.println("Done. Next Values:");
  //Serial.println(Meas_Voltage, 10);
}


  




