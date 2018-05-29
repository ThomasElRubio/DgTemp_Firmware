#include <i2c_t3.h>
#include <Demo2222AB.h>
#include <SPI.h>
#include <AD5760.h>




#define MCLK        4
#define Sync        6

// 1057 = 50µA
//20AF = 100µA


uint32_t code;
float ADC_Vref = 5.0;





void setup(){
  Serial.begin(115200);
  pinMode(Sync,OUTPUT);
  SIM_CLKDIV3 = SIM_CLKDIV3 & 0xFFFFFFF0;
  //SIM_SOPT2[TPMSRC]=0b01; 
  //SIM_SOPT2[PLLFLLSEL] =0b01; //Sets Clock to Phase locked Mode
  output_low(Sync);
  //sneaker_port_init(Sync,CONFIG_DF_16384);
  //SPI.usingInterrupt(2);
  //attachInterrupt(digitalPinToInterrupt(2), Interrupt, FALLING);
  //attachInterrupt(digitalPinToInterrupt(8), Pulse, FALLING);
  Serial.println("Ready");
  Serial.println(SIM_CLKDIV3);
 
}


void loop(){  

}





  




