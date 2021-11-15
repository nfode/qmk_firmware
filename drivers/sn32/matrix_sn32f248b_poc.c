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

#include <stdint.h>
#include <stdbool.h>
#include <SN32F240B.h>
#include "ch.h"
#include "hal.h"
#include "CT16.h"

#include "color.h"
#include "wait.h"
#include "util.h"
#include "matrix.h"
#include "debounce.h"
#include "quantum.h"

static const pin_t row_pins[MATRIX_ROWS] = MATRIX_ROW_PINS;
static const pin_t col_pins[MATRIX_COLS] = MATRIX_COL_PINS;
static const pin_t led_row_pins[LED_MATRIX_ROWS_HW] = LED_MATRIX_ROW_PINS;
static const pin_t led_col_pins[LED_MATRIX_COLS] = LED_MATRIX_COL_PINS;
static uint16_t row_ofsts[LED_MATRIX_ROWS];
static uint8_t mr_offset[24] = {0};
//static uint32_t pwm_en_msk = 0;

#if !defined(PRESSED_KEY_PIN_STATE)
#define PRESSED_KEY_PIN_STATE 0
#endif

matrix_row_t raw_matrix[MATRIX_ROWS]; //raw values
matrix_row_t last_matrix[MATRIX_ROWS] = {0};  // raw values
matrix_row_t matrix[MATRIX_ROWS]; //debounced values

static bool matrix_changed = false;
static uint8_t current_row = 0;
extern volatile LED_TYPE led_state[LED_MATRIX_ROWS * LED_MATRIX_COLS];

__attribute__((weak)) void matrix_init_kb(void) { matrix_init_user(); }

__attribute__((weak)) void matrix_scan_kb(void) { matrix_scan_user(); }

__attribute__((weak)) void matrix_init_user(void) {}

__attribute__((weak)) void matrix_scan_user(void) {}

inline matrix_row_t matrix_get_row(uint8_t row) { return matrix[row]; }

void matrix_print(void) {}

//should give 1000/12000000Mhz = 83us delay
void delay(void){
    for (int i = 0; i < 1000; ++i) {
        __asm__ volatile("" ::: "memory");
    }
}

static void init_pins(void) {
#if(DIODE_DIRECTION == ROW2COL)
    //  Unselect ROWs
    for (uint8_t x = 0; x < MATRIX_ROWS; x++) {
        setPinInput(row_pins[x]);
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

   for (uint8_t x = 0; x < LED_MATRIX_ROWS_HW; x++) {
        setPinOutput(led_row_pins[x]);
        writePinHigh(led_row_pins[x]);
   }
}

/* PWM configuration structure. We use timer CT16B1 with 24 channels. */
static PWMConfig pwmcfg = {
    4000000,       /* PWM clock frequency. */
    256,           /* PWM period (in ticks) 1S (1/10kHz=0.1mS 0.1ms*10000 ticks=1S) */
    NULL,          /* RGB Callback */
    {              /* Default all channels to disabled - Channels will be configured durring init */
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0},
        {PWM_OUTPUT_DISABLED, NULL, 0}
    },
    0/* HW dependent part.*/
};

void rgb_ch_ctrl(PWMConfig *cfg) {

    /* Enable PWM function, IOs and select the PWM modes for the LED column pins */
    for(uint8_t i = 0; i < LED_MATRIX_COLS; i++) {
        switch(led_col_pins[i]) {
            // Intentional fall-through for the PWM B-pin mapping
            case B8:
                cfg->channels[0].pfpamsk = 1;
            case A0:
                cfg->channels[0].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[0] = i;
                break;

            case B9:
                cfg->channels[1].pfpamsk = 1;
            case A1:
                cfg->channels[1].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[1] = i;
                break;

            case B10:
                cfg->channels[2].pfpamsk = 1;
            case A2:
                cfg->channels[2].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[2] = i;
                break;

            case B11:
                cfg->channels[3].pfpamsk = 1;
            case A3:
                cfg->channels[3].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[3] = i;
                break;

            case B12:
                cfg->channels[4].pfpamsk = 1;
            case A4:
                cfg->channels[4].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[4] = i;
                break;

            case B13:
                cfg->channels[5].pfpamsk = 1;
            case A5:
                cfg->channels[5].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[5] = i;
                break;

            case B14:
                cfg->channels[6].pfpamsk = 1;
            case A6:
                cfg->channels[6].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[6] = i;
                break;

            case B15:
                cfg->channels[7].pfpamsk = 1;
            case A7:
                cfg->channels[7].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[7] = i;
                break;

            case C0:
                cfg->channels[8].pfpamsk = 1;
            case A8:
                cfg->channels[8].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[8] = i;
                break;

            case C1:
                cfg->channels[9].pfpamsk = 1;
            case A9:
                cfg->channels[9].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[9] = i;
                break;

            case C2:
                cfg->channels[10].pfpamsk = 1;
            case A10:
                cfg->channels[10].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[10] = i;
                break;

            case C3:
                cfg->channels[11].pfpamsk = 1;
            case A11:
                cfg->channels[11].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[11] = i;
                break;

            case C4:
                cfg->channels[12].pfpamsk = 1;
            case A12:
                cfg->channels[12].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[12] = i;
                break;

            case C5:
                cfg->channels[13].pfpamsk = 1;
            case A13:
                cfg->channels[13].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[13] = i;
                break;

            case C6:
                cfg->channels[14].pfpamsk = 1;
            case A14:
                cfg->channels[14].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[14] = i;
                break;

            case C7:
                cfg->channels[15].pfpamsk = 1;
            case A15:
                cfg->channels[15].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[15] = i;
                break;

            case C8:
                cfg->channels[16].pfpamsk = 1;
            case B0:
                cfg->channels[16].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[16] = i;
                break;

            case C9:
                cfg->channels[17].pfpamsk = 1;
            case B1:
                cfg->channels[17].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[17] = i;
                break;

            case C10:
                cfg->channels[18].pfpamsk = 1;
            case B2:
                cfg->channels[18].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[18] = i;
                break;

            case C11:
                cfg->channels[19].pfpamsk = 1;
            case B3:
                cfg->channels[19].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[19] = i;
                break;

            case C12:
                cfg->channels[20].pfpamsk = 1;
            case B4:
                cfg->channels[20].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[20] = i;
                break;

            case C13:
                cfg->channels[21].pfpamsk = 1;
            case B5:
                cfg->channels[21].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[21] = i;
                break;

            case C14:
                cfg->channels[22].pfpamsk = 1;
            case B6:
                cfg->channels[22].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[22] = i;
                break;

            case C15:
                cfg->channels[23].pfpamsk = 1;
            case B7:
                cfg->channels[23].mode = PWM_OUTPUT_ACTIVE_HIGH;
                mr_offset[23] = i;
                break;
        }
    }

}

void rgb_callback(PWMDriver *pwmp);

void shared_matrix_enable(void) {
    pwmcfg.callback = rgb_callback;
    pwmEnablePeriodicNotification(&PWMD1);
}

void shared_matrix_disable(void) {
    pwmcfg.callback = NULL;
    pwmDisablePeriodicNotification(&PWMD1);
}

void matrix_init(void) {
    // initialize key pins
    init_pins();

    // initialize matrix state: all keys off
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        raw_matrix[i] = 0;
        matrix[i]     = 0;
    }

    for (uint8_t i = 0; i < LED_MATRIX_ROWS; i++) {
        row_ofsts[i] = i * LED_MATRIX_COLS;
    }

    debounce_init(MATRIX_ROWS);

    matrix_init_quantum();

    rgb_ch_ctrl(&pwmcfg);
    pwmStart(&PWMD1, &pwmcfg);
    shared_matrix_enable();

    //NVIC_ClearPendingIRQ(CT16B1_IRQn);
    //nvicEnableVector(CT16B1_IRQn, 0);
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

void rgb_callback(PWMDriver *pwmp) {
    // Disable PWM outputs on column pins
    for(uint8_t i=0;i<24;i++){
        pwmDisableChannel(pwmp,i);
    }
    // Turn the selected row off
    writePinLow(led_row_pins[current_row]);

    // Turn the next row on
    current_row = (current_row + 1) % LED_MATRIX_ROWS_HW;

#if(DIODE_DIRECTION == ROW2COL)
    if(current_row == 0)
    {
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
    }
#elif(DIODE_DIRECTION == COL2ROW)
    if(current_row == 0)
    {
        // Set all column pins input high
        for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
            setPinInputHigh(col_pins[col_index]);
        }
        #if defined(DELAY_ENABLE)
        delay();
        #endif

        // Read the key matrix
        for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
            // Enable the row
            writePinLow(row_pins[row_index]);
            #if defined(DELAY_ENABLE)
            delay();
            #endif

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

        // Set all column pins input low to activate leds
        for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
            setPinInputLow(col_pins[col_index]);
        }
        #if defined(DELAY_ENABLE)
        delay();
        #endif

    }
#endif

    uint8_t row_idx = ( current_row / 3 );
    uint16_t row_ofst = row_ofsts[row_idx];

    if(current_row % 3 == 0)
    {
        for(uint8_t i=0; i<24; i++){
            pwmEnableChannel(pwmp,i,led_state[row_ofst + mr_offset[i] ].r);
        }
    }

    if(current_row % 3 == 1)
    {
        for(uint8_t i=0; i<24; i++){
            pwmEnableChannel(pwmp,i,led_state[row_ofst + mr_offset[i] ].b);
        }
    }

    if(current_row % 3 == 2)
    {
        for(uint8_t i=0; i<24; i++){
            pwmEnableChannel(pwmp,i,led_state[row_ofst + mr_offset[i] ].g);
        }
    }

    writePinHigh(led_row_pins[current_row]);

    //we should not forget to port this optimization as well
//    //optimization to turn off led at lowest brightness
//    //uint32_t new_pwm_en = 0;
//    //if(SN_CT16B1->MR0 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM0EN_EN;
//    //}
//    //if(SN_CT16B1->MR1 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM1EN_EN;
//    //}
//    //if(SN_CT16B1->MR2 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM2EN_EN;
//    //}
//    //if(SN_CT16B1->MR3 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM3EN_EN;
//    //}
//    //if(SN_CT16B1->MR4 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM4EN_EN;
//    //}
//    //if(SN_CT16B1->MR5 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM5EN_EN;
//    //}
//    //if(SN_CT16B1->MR6 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM6EN_EN;
//    //}
//    //if(SN_CT16B1->MR7 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM7EN_EN;
//    //}
//    //if(SN_CT16B1->MR8 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM8EN_EN;
//    //}
//    //if(SN_CT16B1->MR9 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM9EN_EN;
//    //}
//    //if(SN_CT16B1->MR10 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM10EN_EN;
//    //}
//    //if(SN_CT16B1->MR11 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM11EN_EN;
//    //}
//    //if(SN_CT16B1->MR12 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM12EN_EN;
//    //}
//    //if(SN_CT16B1->MR13 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM13EN_EN;
//    //}
//    //if(SN_CT16B1->MR14 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM14EN_EN;
//    //}
//    //if(SN_CT16B1->MR15 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM15EN_EN;
//    //}
//    //if(SN_CT16B1->MR16 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM16EN_EN;
//    //}
//    //if(SN_CT16B1->MR17 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM17EN_EN;
//    //}
//    //if(SN_CT16B1->MR18 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM18EN_EN;
//    //}
//    //if(SN_CT16B1->MR19 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM19EN_EN;
//    //}
//    //if(SN_CT16B1->MR20 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM20EN_EN;
//    //}
//    //if(SN_CT16B1->MR21 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM21EN_EN;
//    //}
//    //if(SN_CT16B1->MR22 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM22EN_EN;
//    //}
//    //if(SN_CT16B1->MR23 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM23EN_EN;
//    //}
//    //
//    //SN_CT16B1->PWMIOENB = pwm_en_msk & new_pwm_en;
//    //SN_CT16B1->PWMIOENB = pwm_en_msk;
}


/**
 * @brief   CT16B1 interrupt handler.
 *
 * @isr
 */
//OSAL_IRQ_HANDLER(SN32_CT16B1_HANDLER) {
//
//    chSysDisable();
//
//    OSAL_IRQ_PROLOGUE();
//
//    PWMDriver *pwmp = &PWMD1;
//    // Disable PWM outputs on column pins
//    //SN_CT16B1->PWMIOENB = 0;
//    for(uint8_t i=0;i<24;i++){
//        pwmDisableChannel(pwmp,i);
//    }
//
//    SN_CT16B1->IC = mskCT16_MR24IC; // Clear match interrupt status
//
//    // Turn the selected row off
//    writePinLow(led_row_pins[current_row]);
//
//    // Turn the next row on
//    current_row = (current_row + 1) % LED_MATRIX_ROWS_HW;
//
//#if(DIODE_DIRECTION == ROW2COL)
//    if(current_row == 0)
//    {
//        // Read the key matrix
//        for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
//            // Enable the column
//            writePinLow(col_pins[col_index]);
//
//            for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
//                // Check row pin state
//                if (readPin(row_pins[row_index]) == PRESSED_KEY_PIN_STATE) {
//                    // Pin LO, set col bit
//                    raw_matrix[row_index] |= (MATRIX_ROW_SHIFTER << col_index);
//                } else {
//                    // Pin HI, clear col bit
//                    raw_matrix[row_index] &= ~(MATRIX_ROW_SHIFTER << col_index);
//                }
//            }
//
//            // Disable the column
//            writePinHigh(col_pins[col_index]);
//        }
//    }
//#elif(DIODE_DIRECTION == COL2ROW)
//    if(current_row == 0)
//    {
//        // Set all column pins input high
//        for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
//            setPinInputHigh(col_pins[col_index]);
//        }
//        #if defined(DELAY_ENABLE)
//        delay();
//        #endif
//
//        // Read the key matrix
//        for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
//            // Enable the row
//            writePinLow(row_pins[row_index]);
//            #if defined(DELAY_ENABLE)
//            delay();
//            #endif
//
//            for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
//                // Check row pin state
//                if (readPin(col_pins[col_index]) == PRESSED_KEY_PIN_STATE) {
//                    // Pin LO, set col bit
//                    raw_matrix[row_index] |= (MATRIX_ROW_SHIFTER << col_index);
//                } else {
//                    // Pin HI, clear col bit
//                    raw_matrix[row_index] &= ~(MATRIX_ROW_SHIFTER << col_index);
//                }
//            }
//
//            // Disable the row
//            writePinHigh(row_pins[row_index]);
//        }
//
//        // Set all column pins input low to activate leds
//        for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
//            setPinInputLow(col_pins[col_index]);
//        }
//        #if defined(DELAY_ENABLE)
//        delay();
//        #endif
//
//    }
//#endif
//
//    uint8_t row_idx = ( current_row / 3 );
//    uint16_t row_ofst = row_ofsts[row_idx];
//
//    if(current_row % 3 == 0)
//    {
//        for(uint8_t i=0; i<24; i++){
//            pwmEnableChannel(pwmp,i,led_state[row_ofst + mr_offset[i] ].r);
//        }
//    }
//
//    if(current_row % 3 == 1)
//    {
//        for(uint8_t i=0; i<24; i++){
//            pwmEnableChannel(pwmp,i,led_state[row_ofst + mr_offset[i] ].b);
//        }
//    }
//
//    if(current_row % 3 == 2)
//    {
//        for(uint8_t i=0; i<24; i++){
//            pwmEnableChannel(pwmp,i,led_state[row_ofst + mr_offset[i] ].g);
//        }
//    }
//
//    //optimization to turn off led at lowest brightness
//    //uint32_t new_pwm_en = 0;
//    //if(SN_CT16B1->MR0 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM0EN_EN;
//    //}
//    //if(SN_CT16B1->MR1 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM1EN_EN;
//    //}
//    //if(SN_CT16B1->MR2 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM2EN_EN;
//    //}
//    //if(SN_CT16B1->MR3 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM3EN_EN;
//    //}
//    //if(SN_CT16B1->MR4 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM4EN_EN;
//    //}
//    //if(SN_CT16B1->MR5 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM5EN_EN;
//    //}
//    //if(SN_CT16B1->MR6 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM6EN_EN;
//    //}
//    //if(SN_CT16B1->MR7 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM7EN_EN;
//    //}
//    //if(SN_CT16B1->MR8 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM8EN_EN;
//    //}
//    //if(SN_CT16B1->MR9 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM9EN_EN;
//    //}
//    //if(SN_CT16B1->MR10 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM10EN_EN;
//    //}
//    //if(SN_CT16B1->MR11 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM11EN_EN;
//    //}
//    //if(SN_CT16B1->MR12 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM12EN_EN;
//    //}
//    //if(SN_CT16B1->MR13 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM13EN_EN;
//    //}
//    //if(SN_CT16B1->MR14 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM14EN_EN;
//    //}
//    //if(SN_CT16B1->MR15 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM15EN_EN;
//    //}
//    //if(SN_CT16B1->MR16 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM16EN_EN;
//    //}
//    //if(SN_CT16B1->MR17 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM17EN_EN;
//    //}
//    //if(SN_CT16B1->MR18 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM18EN_EN;
//    //}
//    //if(SN_CT16B1->MR19 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM19EN_EN;
//    //}
//    //if(SN_CT16B1->MR20 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM20EN_EN;
//    //}
//    //if(SN_CT16B1->MR21 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM21EN_EN;
//    //}
//    //if(SN_CT16B1->MR22 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM22EN_EN;
//    //}
//    //if(SN_CT16B1->MR23 > 0)
//    //{
//    //    new_pwm_en |= mskCT16_PWM23EN_EN;
//    //}
//    //
//    //SN_CT16B1->PWMIOENB = pwm_en_msk & new_pwm_en;
//    //SN_CT16B1->PWMIOENB = pwm_en_msk;
//
//    // Set match interrupts and TC rest
//    //SN_CT16B1->MCTRL3 = (mskCT16_MR24IE_EN | mskCT16_MR24STOP_EN);
//
//    writePinHigh(led_row_pins[current_row]);
//
//    //Set CT16B1 as the up-counting mode.
//    SN_CT16B1->TMRCTRL = (mskCT16_CRST);
//
//    // Wait until timer reset done.
//    while (SN_CT16B1->TMRCTRL & mskCT16_CRST);
//
//    // Let TC start counting.
//    SN_CT16B1->TMRCTRL |= mskCT16_CEN_EN;
//
//    chSysEnable();
//
//    OSAL_IRQ_EPILOGUE();
//}
