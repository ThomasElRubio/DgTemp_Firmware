#ifndef _DEMO2222AB_h
#define _DEMO2222AB_h


#define SNEAKER_PORT_ADDRESS	0x20
#define CONFIG_DF_256     	0x8000
#define CONFIG_DF_1024      0x8400
#define CONFIG_DF_4096      0x8200
#define CONFIG_DF_16384     0x8600
#define CS_LOW          0x00

#define output_high(pin)	digitalWrite(pin, HIGH)
#define output_low(pin)		digitalWrite(pin, LOW)
//#define LTC2508				SPISettings(1000000, MSBFIRST, SPI_MODE0)




#include <Arduino.h>
#include <i2c_t3.h>
#include <SPI.h>

//! Calculates the output voltage from the given digital code and reference voltage
float Code_to_Voltage(int32_t code, float vref);

//! Send n num of pulses on pin given
void send_pulses(uint8_t pin, uint8_t Sync_pin, uint16_t num_of_pulses);

//! Reads 5 bytes of data on SPI - D31:D0 + W7:W0
uint32_t Read_32_Bits(int MCLK);

//! Reads 3 bytes of data with 14bit Signal and 8bit reference Signal
uint16_t Read_14_Bits();

//! Transfers Block of data
void spi_transfer_block(uint8_t *tx, uint8_t *rx, uint8_t data_length);

//! Initializes Sneaker Port to set DF 
void sneaker_port_init(uint8_t Sync_pin, uint16_t global_config_data);

//! Initializes data for Sneaker Port
void initialise_i2c_data(uint16_t value, uint8_t i2c_data[48]);






#endif
