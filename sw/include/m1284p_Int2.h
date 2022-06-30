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

#ifndef M1284P_INT2_H
#define M1284P_INT2_H

#include <avr/interrupt.h>
#include "register_access.h"
#include "M1284P_gpio.h"

namespace m1284p
{
    /**
    @brief Register-level driver for external interrupt INT1
    */
    class Int2
    {
        public:

        ///@brief Interrupt Sense Control
        enum class InterruptSenseControl : uint8_t
        {
            PIN_LOW = 0b00,
            PIN_CHANGE = 0b01,
            FALLING_EDGE = 0b10,
            RISING_EDGE = 0b11
        } ;

        /**
        @brief Initialization
        @param interruptSenseControl Interrupt sense control. Default is interrupt on rising edge
        @param enable Interrupt enable flag. Default is interrupt enabled
        */
        static void init(
        const InterruptSenseControl interruptSenseControl = InterruptSenseControl::RISING_EDGE,
        const bool enable = true)
        {
            Pin::setAsInput();
            ISC::write(interruptSenseControl);
            INT::write(enable);
        }
        
        /**
        @brief Enable Interrupt
        */
        static void enableInterrupt() __attribute__((always_inline))
        {
            // Set interrupt enable flag
            INT::set();
        }
        
        /**
        @brief Disable Interrupt
        */
        static void disableInterrupt() __attribute__((always_inline))
        {
            // Clear interrupt enable flag
            INT::clear();
        }
        
        private:

        // Interrupt Sense Control
        typedef BitGroupInRegister<EICRA, ISC20, ISC21, InterruptSenseControl> ISC;

        // External Interrupt Request Enable
        typedef BitInRegister<EIMSK, INT2> INT;

        // INT1 Pin PD3
        typedef GPIOPin<Port::B, 2> Pin;

        /**
        @brief INT1 interrupt handler
        @note This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleInterrupt() asm("__vector_3") __attribute__((signal, used, externally_visible));
    };
}

#endif