/*    
 *  The AD5781 library is free software: you can redistribute it and/or
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
#ifndef __ARDUINO_AD5680
#define __ARDUINO_AD5680

// #define ARDUINO_AD5762R_DEBUG

#include <Arduino.h>
// include the SPI library:
#include <SPI.h>

class AD5680 {
  public:
    AD5680(uint8_t cs_pin);
    void setValue(uint32_t value);
    void begin();
	void writeSPI(uint32_t value);

  protected:

    // The AD5680 needs just 16 bits of data so it needs to be shifted to left left by 4 bits
    static const uint32_t DAC_REGSISTER_VALUE_OFFSET = 2;
	static const uint32_t RESERVED_BITS = 0b0000 << 20;


    //void writeSPI(uint32_t value);

    uint8_t cs_pin;   // The ~Chip select pin used to address the DAC
    //int16_t ldac_pin;   // The ~LDAC select pin used to address the DAC
   
    SPISettings spi_settings;
};

#endif
