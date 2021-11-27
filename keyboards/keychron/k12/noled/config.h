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

#define MANUFACTURER KeychronX
#define PRODUCT      \x43\x32 WhiteX
#define DESCRIPTION  \x43\x32 White QMKX

#define WAIT_FOR_USB
#define USB_MAX_POWER_CONSUMPTION 100

/* key matrix size */
#define MATRIX_ROWS 5
#define MATRIX_COLS 14

#define DIODE_DIRECTION COL2ROW
#define MATRIX_COL_PINS { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13}
#define MATRIX_ROW_PINS { C4, C5, C6, C7, C8}

/* Debounce reduces chatter (unintended double-presses) - set 0 if debouncing is not needed */
#define DEBOUNCE 0

/* LED Status indicators */
#define LED_CAPS_LOCK_PIN   B3
#define LED_PIN_ON_STATE   1
