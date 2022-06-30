/*
Copyright (C) 2022  Andreas Lagler

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef M1284P_EEPROM_H
#define M1284P_EEPROM_H

#include <stdint.h>
#include <avr/eeprom.h>

namespace m1284p
{
    /**
    @brief Driver for EEMEM (internal EEPROM)
    */
    class EEPROM
    {
        public:
        
        /**
        @brief Data type for address allocated in EEPROM memory
        */
        typedef uint16_t Address;
        
        /**
        @brief Get EEMEM capacity in bytes
        @result EEMEM capacity in bytes
        */
        static constexpr uint32_t capacity()
        {
            return 4096;
        }
        
        /**
        @brief Copy memory for a given number of elements of given type from EEMEM to RAM.
        @tparam Elem Type of the elements to be copied
        @tparam Len Integral length type
        @param dst Destination pointer in EEMEM
        @param src Source pointer in RAM
        @param len length in Elem
        */
        template <typename Elem, typename Len = uint8_t>
        inline void write(Elem * dst, const Elem * src, const Len len)
        {
            eeprom_write_block(
            reinterpret_cast<const uint8_t*>(src),
            reinterpret_cast<uint8_t*>(dst),
            len * sizeof(Elem));
        }

        /**
        @brief Copy memory for one element of given type from EEMEM to RAM.
        @tparam Elem Type of the elements to be copied
        @param dst Destination pointer in EEMEM
        @param src Source pointer in RAM
        */
        template <typename Elem>
        inline void write(Elem * dst, const Elem * src)
        {
            eeprom_write_block(
            reinterpret_cast<const uint8_t*>(src),
            reinterpret_cast<uint8_t*>(dst),
            sizeof(Elem));
        }
        
        /**
        @brief Copy memory for a given number of elements of given type from EEMEM to RAM.
        @tparam Elem Type of the elements to be copied
        @tparam Len Integral length type
        @param dst Destination pointer in RAM
        @param src Source pointer in EEMEM
        @param len length in Elem
        */
        template <typename Elem, typename Len = uint8_t>
        inline void read(Elem * dst, const Elem * src, const Len len)
        {
            eeprom_read_block(
            reinterpret_cast<uint8_t*>(dst),
            reinterpret_cast<const uint8_t*>(src),
            len * sizeof(Elem));
        }

        /**
        @brief Copy memory for one element of given type from EEMEM to RAM.
        @tparam Elem Type of the elements to be copied
        @param dst Destination pointer in RAM
        @param src Source pointer in EEMEM
        */
        template <typename Elem>
        inline void read(Elem * dst, const Elem * src)
        {
            eeprom_read_block(
            reinterpret_cast<uint8_t*>(dst),
            reinterpret_cast<const uint8_t*>(src),
            sizeof(Elem));
        }

        /**
        @brief Write one byte to EEMEM at given position
        @param pos Position in EEMEM (0..4095)
        @param data Byte to be written to EEMEM
        @note This generic write method is compatible with drivers for external memory (which is not memory mapped), so memories are interchangeable
        */
        static void write(const Address pos, const uint8_t data)
        {
            eeprom_write_byte(getBufferPointer(pos), data);
        }

        /**
        @brief Write multiple Bytes to EEMEM starting from given position
        @param pos Position of first byte in EEMEM (0..4095)
        @param data Bytes to be written to EEMEM
        @param nofBytes Number of Bytes to be written to EEMEM (1..4096)
        @note This generic write method is compatible with drivers for external memory (which is not memory mapped), so memories are interchangeable
        */
        static void write(const Address pos, const void * data, const Address nofBytes)
        {
            eeprom_write_block(data, getBufferPointer(pos), nofBytes);
        }
        
        /**
        @brief Read one byte from EEMEM from given position
        @param pos Position in EEMEM (0..4095)
        @result data Byte read from EEMEM
        @note This generic write method is compatible with drivers for external memory (which is not memory mapped), so memories are interchangeable
        */
        [[nodiscard]] static uint8_t read(const Address pos)
        {
            return eeprom_read_byte(getBufferPointer(pos));
        }

        /**
        @brief Read multiple Bytes from EEMEM starting from given position
        @param pos Position of first byte in EEMEM (0..4095)
        @param data Bytes to be read from EEMEM
        @param nofBytes Number of Bytes to be read from EEMEM (1..4096)
        @note This generic write method is compatible with drivers for external memory (which is not memory mapped), so memories are interchangeable
        */
        static void read(const Address pos, void * data, const Address nofBytes)
        {
            eeprom_read_block(data, getBufferPointer(pos), nofBytes);
        }
        
        private:

        // Workaround to avoid an extra cpp file for just the definition of this static buffer
        [[nodiscard]] static uint8_t * getBufferPointer(const Address offset)
        {
            static uint8_t auiBuffer[capacity()] EEMEM;
            return auiBuffer + (offset & (capacity()-1));
        }
    };
}
#endif