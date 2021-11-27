/*
Copyright 2020 Dimitris Mantzouranis

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
*/
#include QMK_KEYBOARD_H

#define KC_TASK LGUI(KC_TAB)        // Task viewer
#define KC_FLXP LGUI(KC_E)          // Windows file explorer
#define KC_CORT LGUI(KC_C)          // Cortana (win10) Microsoft Teams (win11)
#define KC_SNIP LGUI(LSFT(KC_S))    // Windows snip tool
#define KC_W_FN MO(WIN_FN)          // Windows Fn

#define KC_MSSN LGUI(KC_F3)         // Mission Control
#define KC_FIND LALT(LGUI(KC_SPC))  // Finder
#define KC_SIRI LGUI(KC_SPC)        // Siri
#define KC_MSCR LSFT(LGUI(KC_3))    // Mac screenshot
#define KC_MSNP LSFT(LGUI(KC_4))    // Mac snip tool
#define KC_M_FN MO(MAC_FN)          // Mac Fn

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    /*  Windows layout
    +---------------------------------------------------------------------------
    |  ~  |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 |  0 |  - |  = | BACKSP |
    +--------------------------------------------------------------------------+
    |  TAB  |  Q |  W |  E |  R |  T |  Y |  U |  I |  O |  P |  [ |  ] |   \  |
    +--------------------------------------------------------------------------+
    | CAPSLCK  |  A |  S |  D |  F |  G |  H |  J |  K |  L | ; | ' |  RETURN  |
    +--------------------------------------------------------------------------+
    | LSHIFT     |  Z |  X |  C |  V |  B |  N |  M | , | . |  / |   RSHIFT    |
    +--------------------------------------------------------------------------+
    |LCTRL| LGUI| LALT |            SPACE            | RALT| RGUI | FN | RCTRL |
    +---------------------------------------------------------------------------
    */

  [WIN_BASE] = LAYOUT( \
        KC_ESC,   KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     KC_6,     KC_7,     KC_8,     KC_9,     KC_0,     KC_MINS,  KC_EQL,   KC_BSPC, \
        KC_TAB,   KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,     KC_LBRC,  KC_RBRC,  KC_BSLS, \
        KC_CAPS,  KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCLN,  KC_QUOT,            KC_ENT,  \
        KC_LSFT,            KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMM,  KC_DOT,   KC_SLSH,            KC_RSFT, \
        KC_LCTL,   KC_LGUI,  KC_LALT,                                KC_SPC,                                 KC_RALT,  KC_RGUI,  KC_MENU,  KC_RCTL \
    )
   
};
