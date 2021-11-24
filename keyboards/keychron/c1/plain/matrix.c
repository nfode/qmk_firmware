/*
Copyright 2021 Adam Honse <calcprogrammer1@gmail.com>
Copyright 2011 Jun Wako <wakojun@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Ported to QMK by Stephen Peery <https://github.com/smp4488/>
*/

// Key and LED matrix driver for SN32F260.
// This driver will take full control of CT16B1 and GPIO in MATRIX_ROW_PINS, MATRIX_COL_PINS, LED_MATRIX_ROW_PINS and LED_MATRIX_COL_PINS.
//
// The CT16B1 on the SN32F260 can be used as a 22 channel PWM peripheral.
// However PWM cannot be used naively, because a new PWM cycle is started when the previous one ends.
// This is an issue because the key and led matrix is multiplexed and there is some GPIO row and PWM duty cycle reconfiguration needed between PWM cycles.
// The PWM can be stoped and stated again but this has some overhead.
// This driver uses a trick to keep PWM running and have some time in between PWM cycles for the reconfiguration.
//
// While the duty cycle values are between 0-255, the period is configured as 0xFFFF.
// This way the timer period is divide into two phases. 0 to 255 is used as PWM phase, while 256 to 0xFFFF can be used for the reconfiguration.
// The CT16B1 is configured to generate an interupt when the counter reaches 255. In this interrupt the reconfiguration is done.
// The reconfiguration needs some time to execute but not the whole duration from 256 to 0xFFFF.
// So when reconfiguration is done the timer is advanced forward to 0xFFFE.
// One tick later 0xFFFF is reached and a new PWM cycle is started.
//
// The final behaviour is exactly what we want, a PWM cycle followed by a short reconfiguration and then the next PWM cycle.

#include <stdint.h>
#include <stdbool.h>
#include <string.h> // memset()
#include <SN32F260.h>
#include "SN32F260_defines.h"
#include "ch.h"
#include "hal.h"
#include "quantum.h"
#include "matrix.h"
#include "debounce.h"

#ifndef SN32_CT16B1_IRQ_PRIORITY
#define SN32_CT16B1_IRQ_PRIORITY 0
#endif

static const pin_t row_pins[MATRIX_ROWS] = MATRIX_ROW_PINS;
static const pin_t col_pins[MATRIX_COLS] = MATRIX_COL_PINS;
static const pin_t led_row_pins[MATRIX_ROWS] = LED_MATRIX_ROW_PINS;

static matrix_row_t raw_matrix[MATRIX_ROWS]; //raw values
static matrix_row_t last_matrix[MATRIX_ROWS] = {0};  // raw values
static matrix_row_t matrix[MATRIX_ROWS]; //debounced values

static uint8_t current_row = 0;

static inline bool cols_ordered(void);

__attribute__((weak)) void matrix_init_kb(void) { matrix_init_user(); }

__attribute__((weak)) void matrix_scan_kb(void) { matrix_scan_user(); }

__attribute__((weak)) void matrix_init_user(void) {}

__attribute__((weak)) void matrix_scan_user(void) {}

matrix_row_t matrix_get_row(uint8_t row) { return matrix[row]; }

void matrix_print(void) {}

static void init_pins(void) {

#if(DIODE_DIRECTION == ROW2COL)
    //  Unselect ROWs
    for (uint8_t x = 0; x < MATRIX_ROWS; x++) {
        setPinInputHigh(row_pins[x]);
    }

    // Unselect COLs
    for (uint8_t x = 0; x < MATRIX_COLS; x++) {
        setPinOutput(col_pins[x]);
        writePinHigh(col_pins[x]);
    }

#elif(DIODE_DIRECTION == COL2ROW)
    //  Unselect ROWs
    for (uint8_t x = 0; x < MATRIX_ROWS; x++) {
        setPinOutput(row_pins[x]);
        writePinHigh(row_pins[x]);
    }

    // Unselect COLs
    for (uint8_t x = 0; x < MATRIX_COLS; x++) {
        setPinInputHigh(col_pins[x]);
    }
#else
#error DIODE_DIRECTION must be one of COL2ROW or ROW2COL!
#endif

    // Disable the unused LED pins
    for (uint8_t x = 0; x < MATRIX_ROWS; x++) {
        setPinOutput(led_row_pins[x]);
        writePinLow(led_row_pins[x]);
    }
}

void matrix_init(void) {
    // initialize key pins
    init_pins();

    // initialize matrix state: all keys off
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        raw_matrix[i] = 0;
        matrix[i]     = 0;
    }

    debounce_init(MATRIX_ROWS);

    matrix_init_quantum();

    // Enable Timer Clock
    SN_SYS1->AHBCLKEN_b.CT16B1CLKEN = 1;

    // Set match interrupts and TC rest
    SN_CT16B1->MCTRL3 = SN_CT16B1_MCTRL3_MR23IE_Msk |  SN_CT16B1_MCTRL3_MR22RST_Msk;

    // Match register used to generate interrupt.
    SN_CT16B1->MR23 = 0xFF;

    // Match register used to trigger a reset, that will start a new PWM cycle
    SN_CT16B1->MR22 = 0xFFFF;

    // Set prescale value, aim for 1.2kHz. Due to overhead the actual freq will be a bit lower, but should still be above 1kHz.
    // Tests with 12MHz and 6 rows, resulted in a scan frequency of 1.1kHz
    SN_CT16B1->PRE = SystemCoreClock / (256 * MATRIX_ROWS * 1200) - 1;

    //Set CT16B1 as the up-counting mode.
	SN_CT16B1->TMRCTRL = SN_CT16B0_TMRCTRL_CRST_Msk;

    // Wait until timer reset done.
    while (SN_CT16B1->TMRCTRL & SN_CT16B0_TMRCTRL_CRST_Msk);

    // Let TC start counting.
    SN_CT16B1->TMRCTRL = SN_CT16B0_TMRCTRL_CEN_Msk;

    NVIC_ClearPendingIRQ(CT16B1_IRQn);
    nvicEnableVector(CT16B1_IRQn, SN32_CT16B1_IRQ_PRIORITY);
}

uint8_t matrix_scan(void) {
    bool matrix_changed = false;
    for (uint8_t current_col = 0; current_col < MATRIX_COLS; current_col++) {
        for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
            // Determine if the matrix changed state
            if ((last_matrix[row_index] != raw_matrix[row_index])) {
                matrix_changed         = true;
                last_matrix[row_index] = raw_matrix[row_index];
            }
        }
    }

    debounce(raw_matrix, matrix, MATRIX_ROWS, matrix_changed);

    matrix_scan_quantum();

    return matrix_changed;
}

/**
 * @brief   CT16B1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_CT16B1_HANDLER) {
    OSAL_IRQ_PROLOGUE();

    // Clear match interrupt status
    SN_CT16B1->IC = SN_CT16B1_IC_MR22IC_Msk | SN_CT16B1_IC_MR23IC_Msk;

    // Disable PWM outputs on column pins
    SN_CT16B1->PWMIOENB = 0;

    // Move to the next row
    current_row++;
    if(current_row >= MATRIX_ROWS) current_row = 0;

    if(current_row == 0) {

#if(DIODE_DIRECTION == ROW2COL)
        // Delay for the pull ups
        for(int i = 0; i < 20; i++){ __asm__ volatile("" ::: "memory");  }

        // Read the key matrix
        for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
            // Enable the column
            writePinLow(col_pins[col_index]);

            for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
                // Check row pin state
                if (readPin(row_pins[row_index]) == 0) {
                    // Pin LO, set col bit
                    raw_matrix[row_index] |= (MATRIX_ROW_SHIFTER << col_index);
                } else {
                    // Pin HI, clear col bit
                    raw_matrix[row_index] &= ~(MATRIX_ROW_SHIFTER << col_index);
                }
            }

            // Disable the column
            writePinHigh(col_pins[col_index]);
        }

#elif(DIODE_DIRECTION == COL2ROW)
        // Read the key matrix
        for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
            // Enable the row
            writePinLow(row_pins[row_index]);

            // NOTE: Small delay to let internal pull-up resitors pull-up the input pins.
            // The input pin can still been low when the previous row had a pressed button.
            // When transistors are replaced with FETs this is needed because thy have a high gate capacitance.
            for(int i = 0; i < 20; i++){ __asm__ volatile("" ::: "memory");  }

            // When MATRIX_COL_PINS is ordered from MR0 to MRn, the following optimization is possible.
            // This if should be resolved at compile time.
            if( cols_ordered() ){
                // Fast implementation

                // Read all col pin values at once into a varable. This allows for better code gen.
                // The bit position in col_pin_values matches MR numbering.
                uint32_t col_pin_values = (pal_lld_readport(GPIOA) & 0xFFFF) | ((pal_lld_readport(GPIOD) & 0x3F) << 16);

                // Keep bottom MATRIX_COLS bits.
                col_pin_values &= ((1 << MATRIX_COLS) - 1);

                // Invert because a keypress will make a pin low.
                raw_matrix[row_index] = ~col_pin_values;
            } else {
                // Slow fallback implementation

                matrix_row_t raw = 0;
                matrix_row_t mask = 1;
                for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
                    // Check row pin state
                    if (readPin(col_pins[col_index]) == 0) {
                        raw |= mask;
                    }
                    mask <<= 1;
                }

                raw_matrix[row_index] = raw;
            }

            // Disable the row
            writePinHigh(row_pins[row_index]);
        }
#endif

    }

    // Jump to just before MR22 match value, when it is reached it will trigger a reset, starting a new PWM cycle
    SN_CT16B1->TC = 0xFFFE;

    OSAL_IRQ_EPILOGUE();
}

// Map pin to MR number
static inline int8_t pin_to_mr(pin_t pin){
    switch(pin){
        case A0:  return 0;
        case A1:  return 1;
        case A2:  return 2;
        case A3:  return 3;
        case A4:  return 4;
        case A5:  return 5;
        case A6:  return 6;
        case A7:  return 7;
        case A8:  return 8;
        case A9:  return 9;
        case A10: return 10;
        case A11: return 11;
        case A12: return 12;
        case A13: return 13;
        case A14: return 14;
        case A15: return 15;
        case D0:  return 16;
        case D1:  return 17;
        case D2:  return 18;
        case D3:  return 19;
        case D4:  return 20;
        case D5:  return 21;
    }

    // Should not happen!
    return 0;
}

// Return true if MATRIX_COL_PINS is in MR0 to MRn order
// This whole function should optimize to a constant.
static inline bool cols_ordered(void){
    // Tests if MATRIX_COL_PINS[n] maps to MRn
    #define TEST_ORDER(col) if(col < MATRIX_ROWS) { if(col != pin_to_mr(col_pins[col])) return false; }

    // Manual unroll the testing of the order
    TEST_ORDER(0);
    TEST_ORDER(1);
    TEST_ORDER(2);
    TEST_ORDER(3);
    TEST_ORDER(4);
    TEST_ORDER(5);
    TEST_ORDER(6);
    TEST_ORDER(7);
    TEST_ORDER(8);
    TEST_ORDER(9);
    TEST_ORDER(10);
    TEST_ORDER(11);
    TEST_ORDER(12);
    TEST_ORDER(13);
    TEST_ORDER(14);
    TEST_ORDER(15);
    TEST_ORDER(16);
    TEST_ORDER(17);
    TEST_ORDER(18);
    TEST_ORDER(19);
    TEST_ORDER(20);
    TEST_ORDER(21);
    return true;
}
