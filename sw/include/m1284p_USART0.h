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

#ifndef M1284P_USART0_H
#define M1284P_USART0_H

#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include "register_access.h"

namespace m1284p
{
    /**
    @brief Register-level driver class for USART 0 on ATMega1284P
    */
    class USART0
    {
        public:

        ///@brief USART Character Size
        enum class CharacterSize : uint8_t
        {
            _5 = 0b000,
            _6 = 0b001,
            _7 = 0b010,
            _8 = 0b011
            //_9 = 0b111 // CURRENTLY NOT SUPPORTED !!!
        };

        ///@brief USART Mode Select
        enum class Mode : uint8_t
        {
            ASYNC = 0b00,
            SYNC = 0b01,
            SPI_MASTER = 0b11
        };

        ///@brief Parity Mode
        enum class Parity : uint8_t
        {
            NONE = 0b00,
            EVEN = 0b10,
            ODD  = 0b11
        };

        ///@brief Stop Bit Select
        enum class StopBits : uint8_t
        {
            _1 = 0b0,
            _2 = 0b1
        };

        ///@brief Clock Polarity
        enum class ClockPolarity : uint8_t
        {
            OUT_RISING_IN_FALLING = 0b0,
            OUT_FALLING_IN_RISING = 0b1
        };

        /**
        @brief Transmit one Byte of data
        @param data Data byte to transmit
        */
        static void put(const uint8_t data)
        {
            UDR::write(data);
        }

        /**
        @brief Receive one Byte of data
        @result Received data byte
        */
        static uint8_t get()
        {
            return UDR::read();
        }

        /**
        @brief Initialization
        @param cpuClock CPU clock frequency
        @param baudRate baud rate
        @param txEnabled Flag indicating transmitter is enabled
        @param txInterruptEnabled Flag indicating transmission complete interrupt is enabled
        @param UDREInterruptEnabled Flag indicating UDR empty interrupt is enabled
        @param rxEnabled Flag indicating receiver is enabled
        @param rxInterruptEnabled Flag indicating reception complete interrupt is enabled
        @param mode USART mode
        @param characterSize Character size in bits
        @param parity Parity check configuration
        @param stopBits Number of stop bits
        @param clockPolarity Clock polarity
        */
        static void init(
        const uint32_t cpuClock,
        const uint32_t baudRate,
        const bool txEnabled,
        const bool txInterruptEnabled,
        const bool UDREInterruptEnabled,
        const bool rxEnabled,
        const bool rxInterruptEnabled,
        const Mode mode,
        const CharacterSize characterSize,
        const Parity parity,
        const StopBits stopBits,
        const ClockPolarity clockPolarity)
        {
            // Set Baud rate
            UBRR::write(getUBRRValue(cpuClock, baudRate));
            
            // UCSRA
            U2X_Bit::write(getDoubleSpeed(cpuClock, baudRate));
            
            // UCSRB
            RXCIE_Bit::write(rxInterruptEnabled);
            TXCIE_Bit::write(txInterruptEnabled);
            UDRIE_Bit::write(UDREInterruptEnabled);
            RXEN_Bit::write(rxEnabled);
            TXEN_Bit::write(txEnabled);

            // UCSRC
            UMSEL::write(mode);
            UPM::write(parity);
            USBS::write(stopBits);
            UCSZ::write(characterSize);
            UCPOL::write(clockPolarity);
        }
        
        /**
        @brief Start USART transmission
        This method can be used by buffered USART implementations
        */
        static void startTransmission()
        {
            // Transmission is started by enabling the UDRE interrupt
            enableUDREInterrupt();
        }

        /**
        @brief Stop USART transmission
        This method can be used by buffered USART implementations
        */
        static void stopTransmission()
        {
            // Transmission is stopped by disabling the UDRE interrupt
            disableUDREInterrupt();
        }
        
        private:

        // Helper methods to translate the desired baud rate into proper settings for UBBR and the double speed flag
        // All these methods will be evaluated at compile time if the baud rate is a compile time constant        
        
        // Get the UBRR value matching the desired baud rate in single speed
        static constexpr uint16_t getUBRRValueSingleSpeed(const uint32_t clock, const uint32_t baudRate)
        {
            return (clock + baudRate * 8) / (baudRate * 16) - 1;
        }
        
        // Get the UBRR value matching the desired baud rate in double speed
        static constexpr uint16_t getUBRRValueDoubleSpeed(const uint32_t clock, const uint32_t baudRate)
        {
            return (clock + baudRate * 8) / (baudRate * 8) - 1;
        }
        
        // Get real baud rate in single speed using the actual UBRR value calculated from the desired baud rate
        static constexpr uint32_t getRealBaudRateSingleSpeed(const uint32_t clock, const uint32_t baudRate)
        {
            return clock / ( 16 * (getUBRRValueSingleSpeed(clock, baudRate) + 1));
        }
        
        // Get real baud rate in double speed using the actual UBRR value calculated from the desired baud rate
        static constexpr uint32_t getRealBaudRateDoubleSpeed(const uint32_t clock, const uint32_t baudRate)
        {
            return clock / ( 8 * (getUBRRValueDoubleSpeed(clock, baudRate) + 1));
        }
        
        // Get relative baud rate error in single speed
        static constexpr uint32_t getBaudRateErrorSingleSpeed(const uint32_t clock, const uint32_t baudRate)
        {
            if (getRealBaudRateSingleSpeed(clock, baudRate) > baudRate)
            {
                return (getRealBaudRateSingleSpeed(clock, baudRate) * 1000) / baudRate - 1000;
            }
            else
            {
                return (baudRate * 1000) / getRealBaudRateSingleSpeed(clock, baudRate) - 1000;
            }
        }
        
        // Get relative baud rate error in double speed
        static constexpr uint32_t getBaudRateErrorDoubleSpeed(const uint32_t clock, const uint32_t baudRate)
        {
            if (getRealBaudRateDoubleSpeed(clock, baudRate) > baudRate)
            {
                return (getRealBaudRateDoubleSpeed(clock, baudRate) * 1000) / baudRate - 1000;
            }
            else
            {
                return (baudRate * 1000) / getRealBaudRateDoubleSpeed(clock, baudRate) - 1000;
            }
        }
        
        // Set double speed flag if the baud rate error in double speed is lower than in single speed
        static constexpr bool getDoubleSpeed(const uint32_t clock, const uint32_t baudRate)
        {
            return getBaudRateErrorDoubleSpeed(clock, baudRate) < getBaudRateErrorSingleSpeed(clock, baudRate);
        }

        // Get the UBRR value matching the double speed flag
        static constexpr uint16_t getUBRRValue(const uint32_t clock, const uint32_t baudRate)
        {
            if (getDoubleSpeed(clock, baudRate))
            {
                return getUBRRValueDoubleSpeed(clock, baudRate);
            }
            else
            {
                return getUBRRValueSingleSpeed(clock, baudRate);
            }
        }

        // UCSRA

        // USART Receive Complete
        typedef BitInRegister<UCSR0A, RXC0> RXC_Bit;
        
        // USART Transmit Complete
        typedef BitInRegister<UCSR0A, TXC0> TXC_Bit;
        
        // USART Data Register Empty
        typedef BitInRegister<UCSR0A, UDRE0> UDRE_Bit;

        // Frame Error
        typedef BitInRegister<UCSR0A, FE0> FE_Bit;
        
        // Data OverRun
        typedef BitInRegister<UCSR0A, DOR0> DOR_Bit;
        
        // USART Parity Error
        typedef BitInRegister<UCSR0A, UPE0> UPE_Bit;

        // USART double transmission speed
        typedef BitInRegister<UCSR0A, U2X0> U2X_Bit;

        // Multi-processor Communication Mode
        typedef BitInRegister<UCSR0A, MPCM0> MPCM_Bit;

        // UCSRB

        // RX Complete Interrupt Enable
        typedef BitInRegister<UCSR0B, RXCIE0> RXCIE_Bit;

        // RX Complete Interrupt Enable
        typedef BitInRegister<UCSR0B, TXCIE0> TXCIE_Bit;

        // USART Data Register Empty Interrupt Enable
        typedef BitInRegister<UCSR0B, UDRIE0> UDRIE_Bit;
        
        // Receiver Enable
        typedef BitInRegister<UCSR0B, RXEN0> RXEN_Bit;

        // Transmitter Enable
        typedef BitInRegister<UCSR0B, TXEN0> TXEN_Bit;
        
        // UCSRC
        
        // USART Mode Select
        typedef BitGroupInRegister<UCSR0C, UMSEL00, UMSEL01, Mode> UMSEL;
        
        // Parity Mode
        typedef BitGroupInRegister<UCSR0C, UPM00, UPM01, Parity> UPM;
        
        // Stop Bit Select
        typedef BitGroupInRegister<UCSR0C, USBS0, USBS0, StopBits> USBS;
        
        // Character Size
        typedef BitGroupInRegister<UCSR0C, UCSZ00, UCSZ01, CharacterSize> UCSZ;
        
        // Clock Polarity
        typedef BitGroupInRegister<UCSR0C, UCPOL0, UCPOL0, ClockPolarity> UCPOL;

        // UBRR
        typedef UBRR0 UBRR;
        
        // UDR
        typedef UDR0 UDR;

        // USART Control Registers
        typedef UCSR0A UCSRA_Reg;
        typedef UCSR0B UCSRB_Reg;
        typedef UCSR0C UCSRC_Reg;
        
        // Enable UDR empty interrupt
        static void enableUDREInterrupt()
        {
            UDRIE_Bit::set();
        }

        // Disable UDR empty interrupt
        static void disableUDREInterrupt()
        {
            UDRIE_Bit::clear();
        }

        /**
        RX complete interrupt
        This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleRXComplete() __asm__("__vector_20") __attribute__((__signal__, __used__, __externally_visible__));

        /**
        UDR empty interrupt
        This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleUDREmpty() __asm__("__vector_21") __attribute__((__signal__, __used__, __externally_visible__));

        /**
        TX complete interrupt
        This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleTXComplete() __asm__("__vector_22") __attribute__((__signal__, __used__, __externally_visible__));
    };
}

#endif