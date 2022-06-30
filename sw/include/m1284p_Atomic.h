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

#ifndef M1284P_ATOMIC_H
#define M1284P_ATOMIC_H

#include <stdint.h>
#include "register_access.h"

namespace m1284p
{
    /**
    @brief Atomic class which can be used to make code interrupt-safe.
    When an Atomic object is created, the global interrupt flag is stored and cleared.
    When an Atomic object is destroyed, the global interrupt flag is restored.
    
    Usage:
    @code
    // Atomic function body
    {
        Atomic atomic; // Instantiate atomic object to store and clear the global interrupt flag
                
        // interrupt-safe code
        ...
    } // <-- atomic object goes out of scope here and will restore the global interrupt flag    
    @endcode
    */
    class Atomic
    {
        public:

        /**
        @brief Constructor. When an Atomic object is instantiated at the beginning of an atomic section, the Global Interrupt Enabled Flag is stored and then cleared
        */
        Atomic() : m_SREG(SREG::read() & _BV(SREG_I))
        {
            cli();
        }

        /**
        @brief Destructor. When an Atomic object is destroyed at the end of an atomic section, the Global Interrupt Enabled Flag is restored
        */
        ~Atomic()
        {
            SREG::write(SREG::read() | m_SREG);
        }
        
        private:
        
        // Storing the entire SREG is cheaper than storing only the global interrupt bit as a boolean
        const uint8_t m_SREG;
    };
}

#endif