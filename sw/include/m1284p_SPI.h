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

#ifndef M1284P_SPI_H
#define M1284P_SPI_H

#include <stdint.h>
#include "m1284P_GPIO.h"
#include "register_access.h"

namespace m1284p
{
    /**
    @brief Driver for SPI module
    */
    class SPI
    {
        public:

        ///@brief Data Order
        enum class DataOrder : uint8_t
        {
            MSB_FIRST = 0,
            LSB_FIRST = 1
        };

        ///@brief Master/Slave Select
        enum class Mode : uint8_t
        {
            SLAVE = 0,
            MASTER = 1
        };

        ///@brief Clock Polarity
        enum class ClockPolarity : uint8_t
        {
            LOW = 0,
            HIGH = 1
        };

        ///@brief Clock Phase
        enum class ClockPhase : uint8_t
        {
            LEADING = 0,
            TRAILING = 1
        };

        ///@brief SPI Clock Rate Select 1 and 0
        enum class ClockRate : uint8_t
        {
            FOSC_4 = 0b000,
            FOSC_16 = 0b001,
            FOSC_64 = 0b010,
            FOSC_128 = 0b011,
            FOSC_2 = 0b100,
            FOSC_8 = 0b101,
            FOSC_32 = 0b110
        };

        ///@brief Slave Select Pin
        typedef GPIOPin<Port::B, 2> SS_Pin;
        
        /**
        @brief Transmit a single byte
        @param data Byte to be transmitted
        */
        static void transmit(const uint8_t data) __attribute__((always_inline))
        {
            SPDR::write(data);
        }

        /**
        @brief Receive a single byte
        @result Received byte
        */
        static uint8_t receive() __attribute__((always_inline))
        {
            return SPDR::read();
        }

        ///@brief Enable SPI module
        static void enable() __attribute__((always_inline))
        {
            SPE_Bit::set();
        }

        ///@brief Disable SPI module
        static void disable() __attribute__((always_inline))
        {
            SPE_Bit::clear();
        }

        ///@brief Enable SPI Interrupt
        static void enableInterrupt() __attribute__((always_inline))
        {
            SPIE_Bit::set();
        }

        ///@brief Disable SPI Interrupt
        static void disableInterrupt() __attribute__((always_inline))
        {
            SPIE_Bit::clear();
        }

        /**
        @brief Set data order
        @param dataOrder SPI data order
        */
        static void setDataOrder(const DataOrder dataOrder) __attribute__((always_inline))
        {
            DORD_Bit::write(dataOrder);
        }

        /**
        @brief Set clock phase
        @param clockPhase SPI clock phase
        */
        static void setClockPhase(const ClockPhase clockPhase) __attribute__((always_inline))
        {
            CPHA_Bit::write(clockPhase);
        }

        /**
        @brief Set clock polarity
        @param clockPolarity SPI clock polarity
        */
        static void setClockPolarity(const ClockPolarity clockPolarity) __attribute__((always_inline))
        {
            CPOL_Bit::write(clockPolarity);
        }

        /**
        @brief Set clock rate
        @param clockRate SPI clock rate
        */
        static void setClockRate(const ClockRate clockRate) __attribute__((always_inline))
        {
            ClockRate_Bits::write(clockRate);
        }

        ///@brief Initialization in master mode
        static void initMasterMode()
        {
            // In master mode, SS pin is an output pin
            SS_Pin::setAsOutput();
            SS_Pin::high();
            
            // In master mode, SCK pin is an output pin
            SCK_Pin::setAsOutput();
            SCK_Pin::low();

            // In master mode, MOSI pin is an output pin
            MOSI_Pin::setAsOutput();
            MOSI_Pin::low();
            
            // In master mode, MISO pin is an input pin
            MISO_Pin::setAsInput();
            
            // Enable master mode
            MSTR_Bit::write(Mode::MASTER);
        }
        
        ///@brief Initialization in slave mode
        static void initSlaveMode()
        {
            // In slave mode, SS pin is an input pin
            SS_Pin::setAsInput();
            
            // In slave mode, SCK pin is an input pin
            SCK_Pin::setAsInput();

            // In slave mode, MOSI pin is an input pin
            MOSI_Pin::setAsInput();
            
            // In slave mode, MISO pin is an output pin
            MISO_Pin::setAsOutput();
            MISO_Pin::low();
            
            // Enable slave mode
            MSTR_Bit::write(Mode::SLAVE);
  
            // Clear SPI Interrupt Flag by reading SPSR and SPDR
            SPSR::read();
            SPDR::read();
        }
        
        ///@brief Active waiting for transmission complete
        static void wait() __attribute__((always_inline))
        {
            while (!SPIF_Bit::read());
        }

        private:

        // SPI Interrupt Enable
        typedef BitInRegister<SPCR, SPIE> SPIE_Bit;

        // SPI Enable
        typedef BitInRegister<SPCR, SPE> SPE_Bit;

        // Data Order
        typedef BitGroupInRegister<SPCR, DORD, DORD, DataOrder> DORD_Bit;

        // Master/Slave Select
        typedef BitGroupInRegister<SPCR, MSTR, MSTR, Mode> MSTR_Bit;

        // Clock Polarity
        typedef BitGroupInRegister<SPCR, CPOL, CPOL, ClockPolarity> CPOL_Bit;

        // Clock Phase
        typedef BitGroupInRegister<SPCR, CPHA, CPHA, ClockPhase> CPHA_Bit;

        // SPI Clock Rate Select 1 and 0
        typedef BitGroupInRegister<SPCR, SPR0, SPR1> SPR_Bits;

        // SPI Interrupt Flag
        typedef BitInRegister<SPSR, SPIF> SPIF_Bit;

        // Write Collision Flag
        typedef BitInRegister<SPSR, WCOL> WCOL_Bit;

        // Double SPI Speed Bit
        typedef BitInRegister<SPSR, SPI2X> SPI2X_Bit;

        // SPI clock rate logical register bit group (actual bits are spread across two registers)
        struct ClockRate_Bits
        {
            static inline void write(const ClockRate clockRate)
            {
                const union
                {
                    ClockRate clockRate;

                    struct
                    {
                        uint8_t m_SPR:2;
                        bool m_SPI2X:1;
                        uint8_t :5; // Dummy
                    };
                }
                logicalClockRate = {.clockRate = clockRate};
                SPR_Bits::write(logicalClockRate.m_SPR);
                SPI2X_Bit::write(logicalClockRate.m_SPI2X);
            }
        };
        
        // Hardware pins controlled by the SPI module
        typedef GPIOPin<Port::B, 5> MOSI_Pin;
        typedef GPIOPin<Port::B, 6> MISO_Pin;
        typedef GPIOPin<Port::B, 7> SCK_Pin;
        
        /**
        Serial transfer complete interrupt
        This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleTransferComplete() __asm__("__vector_19") __attribute__((__signal__, __used__, __externally_visible__));
    };
}
#endif