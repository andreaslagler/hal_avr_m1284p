/*
Copyright (C) 2022 Andreas Lagler

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef M1284P_GPIO_H
#define M1284P_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "register_access.h"

namespace m1284p
{
    ///@brief GP I/O Port designator
    enum class Port : uint8_t
    {
        A = 0,
        B,
        C,
        D
    };
    
    /**
    @brief GP I/O port SFR definitions
    @tparam t_port Selected GP I/O port
    */
    template <Port t_port>
    class GPIORegisterAccess;

    /**
    @brief GP I/O port SFR definitions for port A
    */
    template <>
    class GPIORegisterAccess<Port::A>
    {
        protected:
        
        /// Output data register
        typedef PORTA PORT;
        
        /// Input data register
        typedef PINA PIN;
        
        /// Data direction register
        typedef DDRA DDR;
    };

    /**
    @brief GP I/O port SFR definitions for port B
    */
    template <>
    class GPIORegisterAccess<Port::B>
    {
        protected:
        
        /// Output data register
        typedef PORTB PORT;
        
        /// Input data register
        typedef PINB PIN;
        
        /// Data direction register
        typedef DDRB DDR;
    };

    /**
    @brief GP I/O port SFR definitions for port C
    */
    template <>
    class GPIORegisterAccess<Port::C>
    {
        protected:
        
        /// Output data register
        typedef PORTC PORT;
        
        /// Input data register
        typedef PINC PIN;
        
        /// Data direction register
        typedef DDRC DDR;
    };

    /**
    @brief GP I/O port SFR definitions for port D
    */
    template <>
    class GPIORegisterAccess<Port::D>
    {
        protected:
        
        /// Output data register
        typedef PORTD PORT;
        
        /// Input data register
        typedef PIND PIN;
        
        /// Data direction register
        typedef DDRD DDR;
    };

    /**
    @brief Register-level driver for GP I/O port
    @tparam t_port Selected GP I/O port
    */
    template <Port t_port>
    class GPIOPort : GPIORegisterAccess<t_port>
    {
        public:
        
        /**
        @brief Set data direction for all pins to input
        */
        static void setAsInput() __attribute__((always_inline))
        {
            DDR::write(0);
        }

        /**
        @brief Set data direction for all pins to output
        */
        static void setAsOutput() __attribute__((always_inline))
        {
            DDR::write(0xFF);
        }

        /**
        @brief Read from port pins
        @result State of the port pins
        @note Set data direction to input beforehand
        */
        [[nodiscard]] static uint8_t read() __attribute__((always_inline))
        {
            return PIN::read();
        }

        /**
        @brief Write to port pins
        @param value Value to be written to the port pins
        @note Set data direction to output beforehand
        */
        static void write(const uint8_t value) __attribute__((always_inline))
        {
            PORT::write(value);
        }
        
        private:
        
        // Redirect register access to base class
        typedef typename GPIORegisterAccess<t_port>::PORT PORT;
        typedef typename GPIORegisterAccess<t_port>::PIN PIN;
        typedef typename GPIORegisterAccess<t_port>::DDR DDR;
    };

    /**
    @brief Register-level driver for a group of pins of a GP I/O port
    @tparam t_port Selected GP I/O port
    @tparam t_firstPin First pin of the pin group (0..7)
    @tparam t_lastPin Last pin of the pin group (0..7)
    */
    template <Port t_port, uint8_t t_firstPin, uint8_t t_lastPin>
    class GPIOSubPort : GPIORegisterAccess<t_port>
    {
        public:

        /**
        @brief Set data direction for the selected pins to input
        */
        static void setAsInput() __attribute__((always_inline))
        {
            DDR::write(0);
        }
        
        /**
        @brief Set data direction for the selected pins to output
        */
        static void setAsOutput() __attribute__((always_inline))
        {
            DDR::write(0xFF);
        }
        
        /**
        @brief Read from selected port pins
        @result State of the selected port pins
        @note Set data direction to input beforehand
        */
        [[nodiscard]] static uint8_t read() __attribute__((always_inline))
        {
            return PIN::read();
        }

        /**
        @brief Write to selected port pins
        @param value Value to be written to the selected port pins
        @note Set data direction to output beforehand
        */
        static void write(const uint8_t value) __attribute__((always_inline))
        {
            PORT::write(value);
        }
        
        /**
        @brief Get number of selected port pins
        @result Number of selected port pins
        */
        static constexpr uint8_t getNofPins()
        {
            return t_lastPin - t_firstPin + 1;
        }
        
        private:
        
        // Redirect register access to base class
        typedef BitGroupInRegister<typename GPIORegisterAccess<t_port>::PORT, t_firstPin, t_lastPin> PORT;
        typedef BitGroupInRegister<typename GPIORegisterAccess<t_port>::PIN, t_firstPin, t_lastPin> PIN;
        typedef BitGroupInRegister<typename GPIORegisterAccess<t_port>::DDR, t_firstPin, t_lastPin> DDR;
    };
    
    /**
    @brief Register-level driver for a single pin of a GP I/O port
    @tparam t_port Selected GP I/O port
    @tparam t_pinIdx Selected pin (0..7)
    */
    template <Port t_port, uint8_t t_pinIdx>
    class GPIOPin : GPIORegisterAccess<t_port>
    {
        public:

        /**
        @brief Set data direction for the selected pin to input
        */
        static void setAsInput() __attribute__((always_inline))
        {
            DDR::clear();
        }

        /**
        @brief Set data direction for the selected pin to output
        */
        static void setAsOutput() __attribute__((always_inline))
        {
            DDR::set();
        }

        /**
        @brief Read from selected port pin
        @result State of the selected port pin
        @note Set data direction to input beforehand
        */
        [[nodiscard]] static bool read() __attribute__((always_inline))
        {
            return PIN::read();
        }

        /**
        @brief Write to selected port pin
        @param value Value to be written to the selected port pin
        @note Set data direction to output beforehand
        */
        static void write(const bool value) __attribute__((always_inline))
        {
            PORT::write(value);
        }

        /**
        @brief Set selected port pin to high
        @note Set data direction to output beforehand
        */
        static void high() __attribute__((always_inline))
        {
            PORT::set();
        }

        /**
        @brief Set selected port pin to low
        @note Set data direction to output beforehand
        */
        static void low() __attribute__((always_inline))
        {
            PORT::clear();
        }

        private:

        // Redirect register access to base class
        typedef BitInRegister<typename GPIORegisterAccess<t_port>::PORT, t_pinIdx> PORT;
        typedef BitInRegister<typename GPIORegisterAccess<t_port>::PIN, t_pinIdx> PIN;
        typedef BitInRegister<typename GPIORegisterAccess<t_port>::DDR, t_pinIdx> DDR;
    };
}

#endif // M1284P_GPIO_H