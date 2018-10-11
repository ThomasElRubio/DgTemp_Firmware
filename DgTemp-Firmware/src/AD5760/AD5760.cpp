/*    
 *  The AD5760 library is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as 
 *  published by the Free Software Foundation, either version 3 of the 
 *  License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AD5760.h"

#define AD5760_SPI_CLOCK_FREQ (1*1000*120)   // 30 MHz SPI clock is the maximum specified
// Use CPOL = 0,  CPHA =1 for ADI DACs

AD5760::AD5760(uint8_t cs_pin) : cs_pin(cs_pin), spi_settings(SPISettings(500000, MSBFIRST, SPI_MODE1)) {
}


// TODO: use an SPI object to select the SPI bus
void AD5760::writeSPI(uint32_t value) {
  SPI.beginTransaction(this->spi_settings);

  digitalWrite (this->cs_pin, LOW);
  
  #ifdef ARDUINO_AD5762R_DEBUG
  Serial.print("Writing to SPI:");
  Serial.println(value, BIN);
  #endif
  
  SPI.transfer((value >> 16) & 0xFF);
  SPI.transfer((value >> 8) & 0xFF);
  SPI.transfer((value >> 0) & 0xFF);

  digitalWrite(this->cs_pin, HIGH);
  SPI.endTransaction();
}

void AD5760::readRegister(uint32_t value){
	
	uint32_t command;
	uint32_t insert ;
	switch(value) {
	case 0b001: insert = this-> DAC_REGISTER; break;
	case 0b010: insert = this-> CONTROL_REGISTER; break;
	case 0b011: insert = this-> CLEARCODE_REGISTER; break;
	default: insert = this->NOP_CONDITION; break;
}
	command = this->READ_REGISTERS | insert | 0x00000;
	//Serial.println(command,BIN);
	this->writeSPI(command);
	this->writeSPI(this->WRITE_REGISTERS | this->NOP_CONDITION | 0x00000);
	
}


void AD5760::updateControlRegister() {
  #ifdef ARDUINO_AD5762R_DEBUG
  Serial.println("Updating control register: ");
  #endif
  this->writeSPI(this->WRITE_REGISTERS | this->CONTROL_REGISTER | this->controlRegister);
  this->writeSPI(this->WRITE_REGISTERS | this->NOP_CONDITION | 0x00000);
}

void AD5760::reset() {
  this->enableOutput();
}

// value is an 18-bit value
void AD5760::setValue(uint32_t value) {
  #ifdef ARDUINO_AD5762R_DEBUG
  Serial.println("Setting DAC value:");
  #endif
  uint32_t command = this->WRITE_REGISTERS | this->DAC_REGISTER | ((value << 4) & 0xFFFFF);

  this->writeSPI(command);
  this->writeSPI(this->WRITE_REGISTERS | this->NOP_CONDITION | 0x00000);
  this->writeSPI(this->SW_CONTROL_REGISTER | this->SW_CONTROL_LDAC);
  this->writeSPI(this->WRITE_REGISTERS | this->NOP_CONDITION | 0x00000);
  
}

void AD5760::enableOutput() {
  #ifdef ARDUINO_AD5762R_DEBUG
  Serial.println("Enabling output.");
  #endif
  this->setOutputClamp(false);
  this->setTristateMode(false);
  //this->setOffsetBinaryEncoding(false);
  this->updateControlRegister();
}

void AD5760::disableOutput() {
  #ifdef ARDUINO_AD5762R_DEBUG
  Serial.println("Enabling output.");
  #endif
  this->setOutputClamp(true);
  this->setTristateMode(true);
  //this->setOffsetBinaryEncoding(true);
  this->updateControlRegister();

}

void AD5760::setInternalAmplifier(bool enable) {
  // (1 << this->RBUF_REGISTER) : internal amplifier is disabled (default)
  // (0 << this->RBUF_REGISTER) : internal amplifier is enabled
  this->controlRegister = (this->controlRegister & ~(1 << this->RBUF_REGISTER)) | (!enable << this->RBUF_REGISTER);
}

// Setting this to enabled will overrule the tristate mode and clamp the output to GND
void AD5760::setOutputClamp(bool enable) {
  // (1 << this->OUTPUT_CLAMP_TO_GND_REGISTER) : the output is clamped to GND (default)
  // (0 << this->OUTPUT_CLAMP_TO_GND_REGISTER) : the dac is in normal mode
  this->controlRegister = (this->controlRegister & ~(1 << this->OUTPUT_CLAMP_TO_GND_REGISTER)) | (enable << this->OUTPUT_CLAMP_TO_GND_REGISTER);
}

void AD5760::setTristateMode(bool enable) {
  // (1 << this->OUTPUT_TRISTATE_REGISTER) : the dac output is in tristate mode (default)
  // (0 << this->OUTPUT_TRISTATE_REGISTER) : the dac is in normal mode
  this->controlRegister = (this->controlRegister & ~(1 << this->OUTPUT_TRISTATE_REGISTER)) | (enable << this->OUTPUT_TRISTATE_REGISTER);
}

void AD5760::setOffsetBinaryEncoding(bool enable) {
  // (1 << this->OFFSET_BINARY_REGISTER) : the dac uses offset binary encoding, should be used when writing unsigned ints
  // (0 << this->OFFSET_BINARY_REGISTER) : the dac uses 2s complement encoding, should be used when writing signed ints (default)
  this->controlRegister = (this->controlRegister & ~(1 << this->OFFSET_BINARY_REGISTER)) | (enable << this->OFFSET_BINARY_REGISTER);
}


/* Linearity error compensation
 * 
 */
// enable = 0 -> Range 0-10 V
// enable = 1 -> Range 0-20 V
void AD5760::setReferenceInputRange(bool enableCompensation) {
  this->controlRegister = (this->controlRegister & ~(0b1111 << this->LINEARITY_COMPENSATION_REGISTER)) | (( this->REFERENCE_RANGE_10V) << this->LINEARITY_COMPENSATION_REGISTER);
}

void AD5760::begin() {
  pinMode(this->cs_pin, OUTPUT);
  digitalWrite(this->cs_pin, HIGH);
  //TODO: add a delay after Setting CS_PIN.
  //this->setOffsetBinaryEncoding(true);
  //this->updateControlRegister();

   //this->reset();
}

