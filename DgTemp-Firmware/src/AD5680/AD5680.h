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

#include <SPI.h>

class AD5680 {
  public:
    AD5680(uint8_t cs_pin, SPIClass* _spi);
    void setValue(const uint32_t value);
    void begin();
    void writeSPI(const uint32_t value);

  protected:
    // The two LSBs are don't care bits, so we need to shift all values
    static const uint32_t DAC_REGSISTER_VALUE_OFFSET = 2;
    uint8_t cs_pin;   // The ~CS pin used to address the DAC
    SPIClass* spi;
    SPISettings spi_settings;
};
#endif

