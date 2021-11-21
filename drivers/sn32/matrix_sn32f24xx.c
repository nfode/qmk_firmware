/*
Copyright 2021 Dimitris Mantzouranis <d3xter93@gmail.com>
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

#include <stdint.h>
#include <stdbool.h>
#include "ch.h"
#include "hal.h"

#include "wait.h"
#include "util.h"
#include "matrix.h"
#include "debounce.h"
#include "quantum.h"
#include "rgb_matrix_sn32f24xx.h"

#ifndef PRESSED_KEY_PIN_STATE
#    define PRESSED_KEY_PIN_STATE 0
#endif

static const pin_t row_pins[MATRIX_ROWS] = MATRIX_ROW_PINS;
static const pin_t col_pins[MATRIX_COLS] = MATRIX_COL_PINS;
matrix_row_t raw_matrix[MATRIX_ROWS]; //raw values
matrix_row_t last_matrix[MATRIX_ROWS] = {0};  // raw values
matrix_row_t matrix[MATRIX_ROWS]; //debounced values

static bool matrix_changed = false;

__attribute__((weak)) void matrix_init_kb(void) { matrix_init_user(); }

__attribute__((weak)) void matrix_scan_kb(void) { matrix_scan_user(); }

__attribute__((weak)) void matrix_init_user(void) {}

__attribute__((weak)) void matrix_scan_user(void) {}

inline matrix_row_t matrix_get_row(uint8_t row) { return matrix[row]; }

void matrix_print(void) {}

static void init_pins(void) {
#if(DIODE_DIRECTION == ROW2COL)
    //  Unselect ROWs
    for (uint8_t x = 0; x < MATRIX_ROWS; x++) {
        setPinInputHigh(row_pins[x]);
        writePinHigh(row_pins[x]);
    }
#elif(DIODE_DIRECTION == COL2ROW)
    //  Unselect ROWs
    for (uint8_t x = 0; x < MATRIX_ROWS; x++) {
        setPinOutput(row_pins[x]);
        writePinHigh(row_pins[x]);
    }
#else
#error DIODE_DIRECTION must be one of COL2ROW or ROW2COL!
#endif

    // Unselect COLs
    for (uint8_t x = 0; x < MATRIX_COLS; x++) {
        setPinOutput(col_pins[x]);
        writePinHigh(col_pins[x]);
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
}

uint8_t matrix_scan(void) {
    matrix_changed = false;
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

void matrix_scan_keys(matrix_row_t raw_matrix[], uint8_t current_row){
        #if(DIODE_DIRECTION == ROW2COL)
            if(current_row == 0)
            {   // Read the key matrix
                for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
                    // Enable the column
                    writePinLow(col_pins[col_index]);
                    for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
                        // Check row pin state
                        if (readPin(row_pins[row_index]) == PRESSED_KEY_PIN_STATE) {
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
            }
        #elif(DIODE_DIRECTION == COL2ROW)
            if(current_row == 0)
            {   // Set all column pins input high
                for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
                    setPinInputHigh(col_pins[col_index]);
                }
                // Read the key matrix
                for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
                    // Enable the row
                    writePinLow(row_pins[row_index]);
                    for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
                        // Check row pin state
                        if (readPin(col_pins[col_index]) == PRESSED_KEY_PIN_STATE) {
                            // Pin LO, set col bit
                            raw_matrix[row_index] |= (MATRIX_ROW_SHIFTER << col_index);
                        } else {
                            // Pin HI, clear col bit
                            raw_matrix[row_index] &= ~(MATRIX_ROW_SHIFTER << col_index);
                        }
                    }
                    // Disable the row
                    writePinHigh(row_pins[row_index]);
                }
            }
        #endif
}