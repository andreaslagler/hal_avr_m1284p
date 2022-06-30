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

#ifndef M1284P_TIMER0_H
#define M1284P_TIMER0_H

#include <stdint.h>
#include <avr/interrupt.h>
#include "register_access.h"


namespace m1284p
{
    /**
    @brief Register-level driver for Timer0 on ATMega1284P
    */
    class Timer0
    {
        public:

        ///@brief Waveform Generation Mode
        enum class WaveformGenerationMode : uint8_t
        {
            NORMAL = 0,             // Normal / TOP: 0xFF / Update of OCRx at: Immediate / TOV Flag Set on: MAX
            PWM_PHASE_CORRECT_1 = 0b001,    // PWM, Phase  Correct / TOP: 0xFF / Update of OCRx at: TOP / TOV Flag Set on: BOTTOM
            CTC = 0b010,            // CTC / TOP: OCRA / Update of OCRx at: Immediate / TOV Flag Set on: MAX
            PWM_FAST_1 = 0b011,     // Fast PWM / TOP: 0xFF / Update of OCRx at: BOTTOM / TOV Flag Set on: MAX
            PWM_PHASE_CORRECT_2 = 0b101,    // PWM, Phase Correct / TOP: OCRA / Update of OCRx at: TOP / TOV Flag Set on: BOTTOM
            PWM_FAST_2 = 0b111      // Fast PWM / TOP: OCRA / Update of OCRx at: BOTTOM / TOV Flag Set on: TOP
        };

        ///@brief Compare Output Mode
        enum class CompareOutputMode : uint8_t
        {
            DISCONNECTED = 0, // Normal port operation, OC0 disconnected.
            TOGGLE = 0b01, // Toggle OC0 on Compare Match
            CLEAR = 0b10, // Clear OC0 on Compare Match
            SET = 0b11  // Set OC0 on Compare Match
        };

        ///@brief Clock Select
        enum class ClockSelect : uint8_t
        {
            NONE = 0, // No clock source (Timer/Counter stopped)
            PRESCALER_1 = 0b001, // clkI/O (No prescaling)
            PRESCALER_8 = 0b010, // clkI/O/8 (From prescaler)
            PRESCALER_64 = 0b011, // clkI/O/64 (From prescaler)
            PRESCALER_256 = 0b100, // clkI/O/256 (From prescaler)
            PRESCALER_1024 = 0b101, // clkI/O/1024 (From prescaler)
            EXT_FALLING = 0b110, // External clock source on T0 pin. Clock on falling edge.
            EXT_RISING = 0b111, // External clock source on T0 pin. Clock on rising edge.
        };

        /**
        @brief Initialization
        @param waveformGenerationMode Selected waveform generation mode for timer 0
        @param clockSelect Selected clock source for timer 0
        @param compareOutputModeA Selected compare output mode for OC0A pin
        @param compareOutputModeB Selected compare output mode for OC0B pin
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

        // Timer/Counter Control Register A
        typedef BitGroupInRegister<TCCR0A, COM0A0, COM0A1, CompareOutputMode> COMA;
        typedef BitGroupInRegister<TCCR0A, COM0B0, COM0B1, CompareOutputMode> COMB;

        // Timer/Counter Control Register B
        typedef BitInRegister<TCCR0B, FOC0A> FOCA;
        typedef BitInRegister<TCCR0B, FOC0B> FOCB;
        typedef BitGroupInRegister<TCCR0B, CS00, CS02, ClockSelect> CS;

        // Timer/Counter Register
        typedef TCNT0 TCNT;
        
        // Output Compare Register A
        typedef OCR0A OCRA;

        // Output Compare Register B
        typedef OCR0B OCRB;

        // Timer/Counter Interrupt Mask Register
        typedef BitInRegister<TIMSK0, OCIE0B> OCIEB;
        typedef BitInRegister<TIMSK0, OCIE0A> OCIEA;
        typedef BitInRegister<TIMSK0, TOIE0> TOIE;

        // Timer/Counter Interrupt Flag Register
        typedef BitInRegister<TIFR0, OCF0B> OCFB;
        typedef BitInRegister<TIFR0, OCF0A> OCFA;
        typedef BitInRegister<TIFR0, TOV0> TOV;

        // Waveform generation mode
        struct WGM
        {
            static void write(const WaveformGenerationMode waveformGenerationMode)
            {
                BitGroupInRegister<TCCR0A, WGM00, WGM01, uint8_t>::write(static_cast<uint8_t>(waveformGenerationMode) & 0b11);
                BitInRegister<TCCR0B, WGM02>::write(static_cast<uint8_t>(waveformGenerationMode) & 0b100);
            }

            static WaveformGenerationMode read()
            {
                uint8_t waveformGenerationMode = BitGroupInRegister<TCCR0A, WGM00, WGM01, uint8_t>::read();
                if (BitInRegister<TCCR0B, WGM02>::read())
                {
                    waveformGenerationMode |= 0b100;
                }
                return static_cast<WaveformGenerationMode>(waveformGenerationMode);
            }
        };
        
        /**
        @brief Compare Match A interrupt handler
        @note This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleCOMPA() __asm__("__vector_16") __attribute__((__signal__, __used__, __externally_visible__));

        /**
        @brief Compare Match B interrupt handler
        @note This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleCOMPB() __asm__("__vector_17") __attribute__((__signal__, __used__, __externally_visible__));

        /**
        @brief Overflow interrupt handler
        @note This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleOVF() __asm__("__vector_18") __attribute__((__signal__, __used__, __externally_visible__));

    };
}

#endif