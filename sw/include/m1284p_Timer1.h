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

#ifndef M1284P_TIMER1_H
#define M1284P_TIMER1_H

#include <stdint.h>
#include <avr/interrupt.h>
#include "register_access.h"


namespace m1284p
{
    /**
    @brief Register-level driver class for Timer1 on ATMega1284P
    */
    class Timer1
    {
        public:

        ///@brief Waveform Generation Mode
        enum class WaveformGenerationMode : uint8_t
        {
            NORMAL = 0,
            PWM_PHASE_CORRECT_8BIT = 0b0001,
            PWM_PHASE_CORRECT_9BIT = 0b0010,
            PWM_PHASE_CORRECT_10BIT = 0b0011,
            CTC_1 = 0b0100,
            PWM_FAST_CORRECT_8BIT = 0b0101,
            PWM_FAST_CORRECT_9BIT = 0b0110,
            PWM_FAST_CORRECT_10BIT = 0b0111,
            PWM_PHASE_CORRECT_FREQ_1 = 0b1000,
            PWM_PHASE_CORRECT_FREQ_2 = 0b1001,
            PWM_PHASE_CORRECT_1 = 0b1010,
            PWM_PHASE_CORRECT_2 = 0b1011,
            CTC_2 = 0b1100,
            PWM_FAST_1 = 0b1110,
            PWM_FAST_2 = 0b1111,
        };

        ///@brief Compare Output Mode
        enum class CompareOutputMode : uint8_t
        {
            DISCONNECTED = 0b00,
            TOGGLE = 0b01,
            CLEAR = 0b10,
            SET = 0b11
        };

        ///@brief Clock Select
        enum class ClockSelect : uint8_t
        {
            NONE = 0,
            PRESCALER_1 = 0b001,
            PRESCALER_8 = 0b010,
            PRESCALER_64 = 0b011,
            PRESCALER_256 = 0b100,
            PRESCALER_1024 = 0b101,
            EXT_FALLING = 0b110,
            EXT_RISING = 0b111,
        };
        
        /**
        @brief Initialization
        @param waveformGenerationMode Selected waveform generation mode for timer 1
        @param clockSelect Selected clock source for timer 1
        @param compareOutputModeA Selected compare output mode for OC1A pin
        @param compareOutputModeB Selected compare output mode for OC1B pin
        */
        static void init(
        const WaveformGenerationMode waveformGenerationMode,
        const ClockSelect clockSelect,
        const CompareOutputMode compareOutputModeA,
        const CompareOutputMode compareOutputModeB)
        {
            WGM::write(waveformGenerationMode);
            CS::write(clockSelect);
            COMA::write(compareOutputModeA);
            COMB::write(compareOutputModeB);
        }
        
        /**
        @brief Enable overflow interrupt
        */
        static void enableOverflowInterrupt() __attribute__((always_inline))
        {
            // Set timer overflow interrupt enable flag
            TOIE::set();
        }
        
        /**
        @brief Disable overflow interrupt
        */
        static void disableOverflowInterrupt() __attribute__((always_inline))
        {
            // Clear timer overflow interrupt enable flag
            TOIE::clear();
        }
        
        private:

        // Compare Output Mode for channel A/B
        typedef BitGroupInRegister<TCCR1A, COM1A0, COM1A1, CompareOutputMode> COMA;
        typedef BitGroupInRegister<TCCR1A, COM1B0, COM1B1, CompareOutputMode> COMB;

        // Input Capture Noise Canceler
        typedef BitInRegister<TCCR1C, ICNC1> ICNC;

        // Input Capture Edge Select
        typedef BitInRegister<TCCR1C, ICES1> ICES;

        // Clock Select
        typedef BitGroupInRegister<TCCR1B, CS10, CS12, ClockSelect> CS;

        // Force Output Compare channel A/B
        typedef BitInRegister<TCCR1C, FOC1A> FOCA;
        typedef BitInRegister<TCCR1C, FOC1B> FOCB;

        // Timer/Counter Register
        typedef TCNT1 TCNT_Reg;
        
        // Output Compare Register A
        typedef OCR1A OCRA_Reg;

        // Output Compare Register B
        typedef OCR1B OCRB_Reg;

        // Input Capture Register
        typedef ICR1 ICR_Reg;
        
        // Timer/Counter Interrupt Mask Register
        typedef BitInRegister<TIMSK1, ICIE1> ICIE;
        typedef BitInRegister<TIMSK1, OCIE1B> OCIEB;
        typedef BitInRegister<TIMSK1, OCIE1A> OCIEA;
        typedef BitInRegister<TIMSK1, TOIE1> TOIE;

        // Timer/Counter 0 Interrupt Flag Register
        typedef BitInRegister<TIFR1, ICF1> ICF;
        typedef BitInRegister<TIFR1, OCF1B> OCFB;
        typedef BitInRegister<TIFR1, OCF1A> OCFA;
        typedef BitInRegister<TIFR1, TOV1> TOV;

        // WGM Waveform generation mode  
        struct WGM
        {
            static void write(const WaveformGenerationMode waveformGenerationMode)
            {
                BitGroupInRegister<TCCR1A, WGM10, WGM11>::write(static_cast<uint8_t>(waveformGenerationMode) & 0b11);
                BitGroupInRegister<TCCR1B, WGM12, WGM13>::write((static_cast<uint8_t>(waveformGenerationMode) & 0b11) << 2);
            }

            static WaveformGenerationMode read()
            {
                uint8_t waveformGenerationMode = BitGroupInRegister<TCCR1B, WGM12, WGM13>::read() << 2;
                waveformGenerationMode |= BitGroupInRegister<TCCR1A, WGM10, WGM11>::read();
                return static_cast<WaveformGenerationMode>(waveformGenerationMode);
            }
        };
        
        /**
        @brief Input Capture interrupt handler
        @note This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleCAPT() __asm__("__vector_12") __attribute__((__signal__, __used__, __externally_visible__));

        /**
        @brief Compare Match A interrupt handler
        @note This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleCOMPA() __asm__("__vector_13") __attribute__((__signal__, __used__, __externally_visible__));

        /**
        @brief Compare Match B interrupt handler
        @note This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleCOMPB() __asm__("__vector_14") __attribute__((__signal__, __used__, __externally_visible__));

       /**
        Overflow interrupt handler
        @note This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleOVF() __asm__("__vector_15") __attribute__((__signal__, __used__, __externally_visible__));
    };
}
#endif