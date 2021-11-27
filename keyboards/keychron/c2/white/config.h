/* Copyright 2020 Adam Honse <calcprogrammer1@gmail.com>
 * Copyright 2020 Dimitris Mantzouranis <d3xter93@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "config_common.h"

/* USB Device descriptor parameter */
#define VENDOR_ID  0x3434
#define PRODUCT_ID 0xFE22
#define DEVICE_VER 0x0001

#define MANUFACTURER Keychron
#define PRODUCT      \x43\x32 White
#define DESCRIPTION  \x43\x32 White QMK

#define WAIT_FOR_USB
#define USB_MAX_POWER_CONSUMPTION 100

/* key matrix size */
#define MATRIX_ROWS 5
#define MATRIX_COLS 14

#define DIODE_DIRECTION ROW2COL
#define MATRIX_COL_PINS { A7, A8, A9, A10, A11, A12, A13, A14, A15, D0, D1, D2, D3, D8}
#define MATRIX_ROW_PINS { C4, C5, C6, C7, C8}


/* LED matrix */
#define LED_MATRIX_ROWS         MATRIX_ROWS
#define LED_MATRIX_ROW_CHANNELS 1
#define LED_MATRIX_ROWS_HW      (LED_MATRIX_ROWS * LED_MATRIX_ROW_CHANNELS)
#define LED_MATRIX_ROW_PINS     { C1, C2, D4, C9, C10 }
#define LED_MATRIX_COLS         MATRIX_COLS
#define LED_MATRIX_COL_PINS     MATRIX_COL_PINS
#define DRIVER_LED_TOTAL        64

/* Backlight configuration */
#define RGB_MATRIX_VAL_STEP            4
#define RGB_DISABLE_WHEN_USB_SUSPENDED true

/* Connects each switch in the dip switch to the GPIO pin of the MCU */
#define DIP_SWITCH_PINS { B3 }

/* Debounce reduces chatter (unintended double-presses) - set 0 if debouncing is not needed */
#define DEBOUNCE 0

/* LED Status indicators */
#define LED_CAPS_LOCK_PIN   D6
#define LED_PIN_ON_STATE   0

#define DYNAMIC_KEYMAP_LAYER_COUNT     4
#define DYNAMIC_KEYMAP_EEPROM_MAX_ADDR 1200

/* Disable the following animation because they are not interesting in monochrome */
#define DISABLE_RGB_MATRIX_ALPHAS_MODS
#define DISABLE_RGB_MATRIX_GRADIENT_UP_DOWN
#define DISABLE_RGB_MATRIX_GRADIENT_LEFT_RIGHT
#define DISABLE_RGB_MATRIX_BAND_SAT
#define DISABLE_RGB_MATRIX_BAND_PINWHEEL_SAT
#define DISABLE_RGB_MATRIX_BAND_SPIRAL_SAT
#define DISABLE_RGB_MATRIX_CYCLE_ALL
#define DISABLE_RGB_MATRIX_RAINBOW_MOVING_CHEVRON
#define DISABLE_RGB_MATRIX_DUAL_BEACON

/* TODO: Disabled the following animation because they crash the keyboard. Probably due to a too small stack */
#define DISABLE_RGB_MATRIX_RAINDROPS
#define DISABLE_RGB_MATRIX_JELLYBEAN_RAINDROPS

/* Enable NKRO by default */
#define FORCE_NKRO
