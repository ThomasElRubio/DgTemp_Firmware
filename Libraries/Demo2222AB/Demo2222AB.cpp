



/*

Serial Userinterface to choose between filtered 32 bits und unfiltered 14 bit output
also make a menu option to choose downsampling-factor DF

*/

#ifndef DEMO2222AB_cpp
#define DEMO2222AB_cpp

#include <SPI.h>
#include <i2c_t3.h>
#include "Demo2222AB.h"
#include "Arduino.h"

//#define global_config_data = CONFIG_DF_256



//! Calculates the output voltage from the given digital code and reference voltage
float Code_to_Voltage(int32_t code, float vref){
	float voltage;
  voltage = ((float)code / 2147483647) * vref;
  return voltage;
	
}



//! Send n num of pulses on pin given
void send_pulses(uint8_t pin ,uint8_t Sync_pin,uint16_t num_of_pulses){
  uint16_t i;
  output_high(Sync_pin);
  delayMicroseconds(1);
  output_low(Sync_pin);
  digitalWriteFast(pin,HIGH);
  //delayMicroseconds(10);
  /*for (i = 0; i < num_of_pulses-1; ++i)
  {
    digitalWrite(pin,LOW);  	// Pull CS low
	delayMicroseconds(10);
    digitalWrite(pin,HIGH);                     // Pull CS high
	delayMicroseconds(10);
  }*/
  //output_low(pin);
  //delayMicroseconds(10);  
}



//! Reads 5 bytes of data on SPI - D31:D0 + W7:W0
uint32_t Read_32_Bits(int MCLK){
	output_low(MCLK);
	uint8_t rx[5];
	uint8_t tx[5] = {0,0,0,0,0};
	uint32_t code = 0;

	
	spi_transfer_block(tx, rx, 5);       // Read 5 bytes on SPI port
	//Serial.print(rx[5],HEX);
	/*Serial.print(rx[4],HEX);
	Serial.print(rx[3],HEX);
	Serial.print(rx[2],HEX);
	Serial.print(rx[1],HEX);*/
	//Serial.print(rx[0],HEX);
	//Serial.print(" , ");
	code = rx[4];
	code = (code << 8) | rx[3];
	code = (code << 8) | rx[2];
	code = (code << 8) | rx[1];
	
	return code;
}



//! Read unfiltered Output
 uint16_t Read_14_Bits(){
	uint8_t rx[3];
	uint8_t tx[3] = {0,0,0};
	uint16_t code = 0;
	uint8_t mode = 0;
	uint16_t byte[2];
	
	spi_transfer_block( tx, rx, 3);
	
	code = rx[2];
	code = (code<<8)| rx[1];
	code = code>>2;
	mode = (rx[1]<<6)+(rx[0]>>2);
	byte[0] = code;
	byte[1] = mode;

  return byte[1];
}	



//! Reads and sends a byte array
void spi_transfer_block(uint8_t *tx, uint8_t *rx, uint8_t data_length){
  int8_t i;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  for (i=(data_length-1);  i >= 0; i--){
    rx[i] = SPI.transfer(tx[i]);//! 2) Read and send byte array
	//Serial.println(rx[i], BIN);
	delayMicroseconds(1);
	}
  SPI.endTransaction();
}


void sneaker_port_init(uint8_t Sync_pin, uint16_t global_config_data){
	uint8_t i;
	uint16_t config_value = global_config_data;
	uint8_t i2c_data[48];                 // 48 bytes of data: 3 bytes each for 16 bits
	int i2c_status;
	
	
	initialise_i2c_data(config_value, i2c_data);      // Populate i2c_data array with values to bit bang P3, P5, P6
	
	Wire.beginTransmission(SNEAKER_PORT_ADDRESS);	
	Wire.write(CS_LOW);
	Serial.println(i2c_status);	
	for(i = 0; i<48;i++){
		Wire.write(i2c_data[i]);
		//Serial.print(i2c_data[i],HEX);
	}
	Wire.write(0x04);	
	Wire.write(0x8c);
	i2c_status = Wire.endTransmission(I2C_STOP);
	Serial.println(i2c_status);
	Serial.println("DONE");

	//Send a SYNC pulse and leave it low

	//output_low(Sync_pin);
	
	digitalWrite(Sync_pin, LOW);   // SYNC is connected to GPIO pin of the QuikEval connector
	digitalWrite(Sync_pin, HIGH);
	digitalWrite(Sync_pin, LOW);
	
}


  
//! Initialize Data for Sneaker Port
void initialise_i2c_data(uint16_t value, uint8_t i2c_data[48]){
  uint8_t i;
  uint8_t set_bit;
  uint8_t *p = i2c_data;
  for (i = 0; i < 16; ++i)
  {
    set_bit = (value >> i) & 0x01;
    if (set_bit)
    {
      *p++ = 0x40;
      *p++ = 0x60;
      *p++ = 0x40;
    }
    else
    {
      *p++ = 0x00;
      *p++ = 0x20;
      *p++ = 0x00;
    }
  }
}  
  
  
  
#endif  
  
