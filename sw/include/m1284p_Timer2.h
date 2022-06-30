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

#ifndef M1284P_TIMER2_H
#define M1284P_TIMER2_H

#include <stdint.h>
#include <avr/interrupt.h>
#include "register_access.h"

namespace m1284p
{
    /**
    @brief Register-level driver class for Timer2 on ATMega1284P
    @note Asynchronous operation is currently not supported!
    */
    class Timer2
    {
        public:

        ///@brief  Waveform Generation Mode
        enum class WaveformGenerationMode : uint8_t
        {
            NORMAL = 0,      // Normal / TOP: 0xFF / Update of OCRx at: Immediate / TOV Flag Set on: MAX
            PWM_PHASE_CORRECT_1 = 0b001, // PWM, Phase  Correct / TOP: 0xFF / Update of OCRx at: TOP / TOV Flag Set on: BOTTOM
            CTC = 0b010,         // CTC / TOP: OCRA / Update of OCRx at: Immediate / TOV Flag Set on: MAX
            PWM_FAST_1 = 0b011,  // Fast PWM / TOP: 0xFF / Update of OCRx at: BOTTOM / TOV Flag Set on: MAX
            PWM_PHASE_CORRECT_2 = 0b101, // PWM, Phase Correct / TOP: OCRA / Update of OCRx at: TOP / TOV Flag Set on: BOTTOM
            PWM_FAST_2 = 0b111   // Fast PWM / TOP: OCRA / Update of OCRx at: BOTTOM / TOV Flag Set on: TOP
        };

        ///@brief  Compare Output Mode
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
            PRESCALER_1 = 0b001, // clk/1(No prescaling)
            PRESCALER_8 = 0b010, // clk/8 (From prescaler)
            PRESCALER_32 = 0b011, // clk/32 (From prescaler)
            PRESCALER_64 = 0b100, // clk/64 (From prescaler)
            PRESCALER_128 = 0b101, // clk/128 (From prescaler)
            PRESCALER_256 = 0b110, // clk/256 (From prescaler)
            PRESCALER_1024 = 0b111 // clk/1024 (From prescaler)
        };

        /**
        @brief Initialization
        @param waveformGenerationMode Selected waveform generation mode for timer 2
        @param clockSelect Selected clock source for timer 2
        @param compareOutputModeA Selected compare output mode for OC2A pin
        @param compareOutputModeB Selected compare output mode for OC2B pin
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

        // TCCR0A – Timer/Counter Control Register A
        typedef BitGroupInRegister<TCCR2A, COM2A0, COM2A1, CompareOutputMode> COMA;
        typedef BitGroupInRegister<TCCR2A, COM2B0, COM2B1, CompareOutputMode> COMB;

        // TCCR0B – Timer/Counter Control Register B
        typedef BitInRegister<TCCR2B, FOC2A> FOCA;
        typedef BitInRegister<TCCR2B, FOC2B> FOCB;
        typedef BitGroupInRegister<TCCR2B, CS20, CS22, ClockSelect> CS;

        // TCNT0 – Timer/Counter Register
        typedef TCNT2 TCNT;
        
        // OCR0A – Output Compare Register A
        typedef OCR2A OCRA;

        // OCR0B – Output Compare Register B
        typedef OCR2B OCRB;

        // TIMSK0 – Timer/Counter Interrupt Mask Register
        typedef BitInRegister<TIMSK2, OCIE2B> OCIEB;
        typedef BitInRegister<TIMSK2, OCIE2A> OCIEA;
        typedef BitInRegister<TIMSK2, TOIE2> TOIE;

        // TIFR0 – Timer/Counter 0 Interrupt Flag Register
        typedef BitInRegister<TIFR2, OCF2B> OCFB;
        typedef BitInRegister<TIFR2, OCF2A> OCFA;
        typedef BitInRegister<TIFR2, TOV2> TOV;

        // WGM Waveform generation mode
        struct WGM
        {
            static void write(const WaveformGenerationMode waveformGenerationMode)
            {
                BitGroupInRegister<TCCR2A, WGM20, WGM21>::write(static_cast<uint8_t>(waveformGenerationMode) & 0b11);
                BitInRegister<TCCR2B, WGM22>::write(static_cast<uint8_t>(waveformGenerationMode) & 0b100);
            }

            static WaveformGenerationMode read()
            {
                uint8_t waveformGenerationMode = BitGroupInRegister<TCCR2A, WGM20, WGM21>::read();
                if (BitInRegister<TCCR2B, WGM22>::read())
                {
                waveformGenerationMode |= 0b100;
                }                
                return static_cast<WaveformGenerationMode>(waveformGenerationMode);
            }
        };

        /**
        Compare Match A interrupt handler
        @note This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleCOMPA() __asm__("__vector_7") __attribute__((__signal__, __used__, __externally_visible__));

        /**
        Compare Match B interrupt handler
        @note This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleCOMPB() __asm__("__vector_8") __attribute__((__signal__, __used__, __externally_visible__));

        /**
        Overflow interrupt handler
        @note This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleOVF() __asm__("__vector_9") __attribute__((__signal__, __used__, __externally_visible__));
        

    };
}
#endif