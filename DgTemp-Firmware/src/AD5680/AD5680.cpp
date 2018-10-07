/*    
 *  The AD5680 library is free software: you can redistribute it and/or
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
#include "AD5680.h"

#define AD5680_SPI_CLOCK_FREQ (1*1000*120)   // 30 MHz SPI clock is the maximum specified
// Use CPOL = 0,  CPHA =1 for ADI DACs

AD5680::AD5680(uint8_t cs_pin) : cs_pin(cs_pin), spi_settings(SPISettings(1000000, MSBFIRST, SPI_MODE1)) {
}


// TODO: use an SPI object to select the SPI bus
void AD5680::writeSPI(uint32_t value) {
  SPI.beginTransaction(this->spi_settings);

  digitalWrite (this->cs_pin, LOW);
  delayMicroseconds(10);
  
  #ifdef ARDUINO_AD5762R_DEBUG
  Serial.print("Writing to SPI:");
  Serial.println(value, BIN);
  #endif
  
  SPI.transfer((value >> 16) & 0xFF);
  delayMicroseconds(4);
  SPI.transfer((value >> 8) & 0xFF);
  delayMicroseconds(4);
  SPI.transfer((value >> 0) & 0xFF);
  delayMicroseconds(10);

  digitalWrite(this->cs_pin, HIGH);
  delayMicroseconds(10);
  SPI.endTransaction();
}




// value is an 18-bit value
void AD5680::setValue(uint32_t value) {
  Serial.print(value);
  Serial.print(",");
  #ifdef ARDUINO_AD5762R_DEBUG
  Serial.println("Setting DAC value:");
  #endif
  uint32_t command = this->RESERVED_BITS | ((value << 2) & 0xFFFFF);
  
  this->writeSPI(command);
  
}


void AD5680::begin() {
  pinMode(this->cs_pin, OUTPUT);
  digitalWrite(this->cs_pin, HIGH);
  delay(10);
}

