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

#ifndef M1284P_ADC_H
#define M1284P_ADC_H

#include <stdint.h>
#include <stdbool.h>
#include "register_access.h"

namespace m1284p
{
    /**
    @brief Register-level driver class for A/D converter on ATMega1284P
    */
    class ADConverter
    {
        public:

        ///@brief Reference Selection Bits
        enum class ReferenceSelection : uint8_t
        {
            AREF = 0b00,
            AVCC = 0b01,
            INTERNAL_1_1_VOLTS = 0b10,
            INTERNAL_2_56_VOLTS = 0b11,
        };

        ///@brief Analog Channel Selection Bits
        enum class ChannelSelection : uint8_t
        {
            ADC0 = 0b00000,
            ADC1 = 0b00001,
            ADC2 = 0b00010,
            ADC3 = 0b00011,
            ADC4 = 0b00100,
            ADC5 = 0b00101,
            ADC6 = 0b00110,
            ADC7 = 0b00111,
            // Differential modes 01000 .. 11100 not supported
            VBG = 0b11110,
            GND = 0b11111
        };

        ///@brief Pre-scaler Selection Bits
        enum class PrescalerSelect : uint8_t
        {
            DIV_2 = 0b000,
            DIV_2_ = 0b001,
            DIV_4 = 0b010,
            DIV_8 = 0b011,
            DIV_16 = 0b100,
            DIV_32 = 0b101,
            DIV_64 = 0b110,
            DIV_128 = 0b111
        };

        ///@brief ADC Auto Trigger Source
        enum class AutoTriggerSource : uint8_t
        {
            FREE_RUN = 0b000,
            COMPARATOR = 0b001,
            INT0_INTERRUPT = 0b010,
            TIMER0_COMPARE_MATCH_A = 0b011,
            TIMER0_OVERFLOW = 0b100,
            TIMER1_COMPARE_MATCH_B = 0b101,
            TIMER1_OVERFLOW = 0b110,
            TIMER1_CAPTURE = 0b111
        };
        
        /**
        @brief Initialization
        @param referenceSelection Selected ADC reference voltage
        @param prescalerSelect Selected ADC clock pre-scaler
        @param interruptEnable Flag indicating ADC interrupt on conversion complete is enabled
        @param autoTriggerEnable Flag indicating ADC is triggered automatically
        @param autoTriggerSource Selected source for auto trigger
        @param enableADC0 Flag indication ADC0 pin is configured as analog input pin,
        @param enableADC1 Flag indication ADC1 pin is configured as analog input pin,
        @param enableADC2 Flag indication ADC2 pin is configured as analog input pin,
        @param enableADC3 Flag indication ADC3 pin is configured as analog input pin,
        @param enableADC4 Flag indication ADC4 pin is configured as analog input pin,
        @param enableADC5 Flag indication ADC5 pin is configured as analog input pin)
        @param enableADC6 Flag indication ADC6 pin is configured as analog input pin,
        @param enableADC7 Flag indication ADC7 pin is configured as analog input pin)
        */
        static void init(
        const ReferenceSelection referenceSelection,
        const PrescalerSelect prescalerSelect,
        const bool interruptEnable,
        const bool autoTriggerEnable,
        const AutoTriggerSource autoTriggerSource,
        const bool enableADC0,
        const bool enableADC1,
        const bool enableADC2,
        const bool enableADC3,
        const bool enableADC4,
        const bool enableADC5,
        const bool enableADC6,
        const bool enableADC7)
        {
            // ADMUX register
            REFS::write(referenceSelection);
            ADLAR_bit::set(); // Left-align conversion result

            // ADC Control and Status Register A
            ADEN_bit::write(true);
            ADATE_bit::write(autoTriggerEnable);
            ADIE_bit::write(interruptEnable);
            ADPS::write(prescalerSelect);

            // ADC Control and Status Register B
            ADTS::write(autoTriggerSource);

            // Digital Input Disable Register 0
            ADC0D_Bit::write(enableADC0);
            ADC1D_Bit::write(enableADC1);
            ADC2D_Bit::write(enableADC2);
            ADC3D_Bit::write(enableADC3);
            ADC4D_Bit::write(enableADC4);
            ADC5D_Bit::write(enableADC5);
            ADC6D_Bit::write(enableADC6);
            ADC7D_Bit::write(enableADC7);
        }

        /**
        @brief Analog input pin driver class implementing high-level ADC access
        @tparam t_channelIdx Corresponding ADC channel index 0..7
        @note The ADC module has to initialized beforehand using the parent ADC driver class interface
        */
        template <uint8_t t_channelIdx>
        class Pin
        {
            public:
            
            /**
            @brief Read the AD conversion result in the desired resolution
            @tparam Result Type of AD conversion result (here: 8 Bit or 16 Bit unsigned)
            @result AD conversion result
            */
            template <typename Result>
            static Result read()
            {
                return ADConverter::template read<Result>();
            }
            
            /**
            @brief Start A/D conversion on the selected pin
            @note This method will wait synchronously until the ADC is ready
            */
            static void startConversion() __attribute__((always_inline))
            {
                select();
                ADConverter::startConversion();
            }
            
            /**
            @brief  Wait synchronously until the ADC is ready
            */
            static void wait() __attribute__((always_inline))
            {
                ADConverter::wait();
            }
            
            private:
            
            // Select the channel on the ADC
            static void select() __attribute__((always_inline))
            {
                static_assert(t_channelIdx <= 8, "Invalid channel: Selected channel index >= 8!");
                ADConverter::selectChannel(static_cast<ChannelSelection>(t_channelIdx));
            }
        };

        private:

        // Read A/D conversion result in the desired resolution
        template <typename Result>
        static Result read();
        
        // Select ADC input channel
        static void selectChannel(const ChannelSelection channelSelection) __attribute__((always_inline))
        {
            // Select ADC channel
            MUX::write(channelSelection);
        }

        // Start A/D conversion
        static void startConversion() __attribute__((always_inline))
        {
            // Start A/D conversion by setting ADSC
            ADSC_bit::set();
        }
        
        // Wait for AD conversion to complete
        static void wait() __attribute__((always_inline))
        {
            // AD conversion is in progress while ADSC is set
            while (ADSC_bit::read());
        }

        // Reference Selection Bits
        typedef BitGroupInRegister<ADMUX, REFS0, REFS1, ReferenceSelection> REFS;

        // ADC Left Adjust Result
        typedef BitInRegister<ADMUX, ADLAR> ADLAR_bit;

        // Analog Channel Selection Bits
        typedef BitGroupInRegister<ADMUX, MUX0, MUX4, ChannelSelection> MUX;

        // ADC Enable
        typedef BitInRegister<ADCSRA, ADEN> ADEN_bit;

        // ADC Start Conversion
        typedef BitInRegister<ADCSRA, ADSC> ADSC_bit;

        // ADC Auto Trigger Enable
        typedef BitInRegister<ADCSRA, ADATE> ADATE_bit;

        // ADC Interrupt Flag
        typedef BitInRegister<ADCSRA, ADIF> ADIF_bit;

        // ADC Interrupt Enable
        typedef BitInRegister<ADCSRA, ADIE> ADIE_bit;

        // ADC Prescaler Select Bits
        typedef BitGroupInRegister<ADCSRA, ADPS0, ADPS2, PrescalerSelect> ADPS;

        // ADC Auto Trigger Source
        typedef BitGroupInRegister<ADCSRB, ADTS0, ADTS2, AutoTriggerSource> ADTS;

        // ADC5..0 Digital Input Disable
        typedef BitInRegister<DIDR0, ADC0D> ADC0D_Bit;
        typedef BitInRegister<DIDR0, ADC1D> ADC1D_Bit;
        typedef BitInRegister<DIDR0, ADC2D> ADC2D_Bit;
        typedef BitInRegister<DIDR0, ADC3D> ADC3D_Bit;
        typedef BitInRegister<DIDR0, ADC4D> ADC4D_Bit;
        typedef BitInRegister<DIDR0, ADC5D> ADC5D_Bit;
        typedef BitInRegister<DIDR0, ADC6D> ADC6D_Bit;
        typedef BitInRegister<DIDR0, ADC7D> ADC7D_Bit;
      
        /**
        ADC interrupt
        This method has to be defined in a separate cpp file. Otherwise, interrupt vector table won't be populated
        */
        static void handleADCInterrupt() __asm__("__vector_24") __attribute__((__signal__, __used__, __externally_visible__));
    };

    // template method specializations
    template<>
    inline uint8_t ADConverter::read<uint8_t>()
    {
        // Read 8 bit result (left-aligned)
        return ADCH::read();
    }

    template<>
    inline uint16_t ADConverter::read<uint16_t>()
    {
        // Read 16 bit result
        return ADC::read();
    }
}

#endif