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

#ifdef ARDUINO_AD5762R_DEBUG
  #include <Arduino.h>
#endif

#include "AD5680.h"

#define AD5680_SPI_CLOCK_FREQ (1*1000*1000)   // 30 MHz SPI clock is the maximum specified
// Use CPOL = 0, CPHA =1 for ADI DACs

AD5680::AD5680(uint8_t cs_pin, SPIClass* _spi) : cs_pin(cs_pin), spi(_spi), spi_settings(SPISettings(AD5680_SPI_CLOCK_FREQ, MSBFIRST, SPI_MODE1)) {};


// TODO: use an SPI object to select the SPI bus
void AD5680::writeSPI(const uint32_t value) {
  this->spi->beginTransaction(this->spi_settings);

  digitalWrite(this->cs_pin, LOW);
  delayMicroseconds(10);

  #ifdef ARDUINO_AD5762R_DEBUG
  Serial.print("Writing to SPI:");
  Serial.println(value, BIN);
  #endif

  // The input shift register is 24 bits wide. So we do not care about the top byte of the 32 bit integer
  this->spi->transfer((value >> 16) & 0xFF);
  delayMicroseconds(4);
  this->spi->transfer((value >> 8) & 0xFF);
  delayMicroseconds(4);
  this->spi->transfer((value >> 0) & 0xFF);
  delayMicroseconds(10);

  digitalWrite(this->cs_pin, HIGH);
  delayMicroseconds(10);
  this->spi->endTransaction();
}

// only the the 18 LSBs of the value will be used
void AD5680::setValue(const uint32_t value) {
  #ifdef ARDUINO_AD5762R_DEBUG
  Serial.print("Setting DAC value:");
  Serial.println(value & 0x3FFFF);
  #endif
  uint32_t command = ((value & 0x3FFFF) << this->DAC_REGSISTER_VALUE_OFFSET);
  
  this->writeSPI(command);
}

void AD5680::begin() {
  pinMode(this->cs_pin, OUTPUT);
  digitalWrite(this->cs_pin, HIGH);
  delay(10);
}

