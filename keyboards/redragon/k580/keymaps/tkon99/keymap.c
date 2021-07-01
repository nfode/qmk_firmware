/*
Copyright 2019 Stephen Peery

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

// Each layer gets a name for readability, which is then used in the keymap matrix below.
// The underscores don't mean anything - you can have a layer called STUFF or any other name.
// Layer names don't all need to be of the same length, obviously, and you can also skip them
// entirely and just use numbers.
enum layer_names {
    _BASE,
    _FN,
};

// K580 Media Keys
// 1,20                 1,19            6,20           4,20
// Vol/Brightness       Media prev.     Play/Pause     Media next

// K580 Macro Keys
// 5,16     5,14    4,16    4,15    4,14    1,13
// G1       G2      G3      G4      G5      Rec

// Macro Keys (for default keymap)
enum {
  U_G1 = SAFE_RANGE,
  U_G2,
  U_G3,
  U_G4,
  U_G5,
  U_VOLBRI // Volume/Brightness toggle
};
bool BRI = false; // Keep track of volume/brightness for the encoder

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    /*  Row:        0           1           2           3        4           5        6        7           8           9           10          11          12          13          14          15          16          17          18          19          20     */
    [_BASE] = { {   KC_ESC,     KC_F1,      KC_F2,      KC_F3,   KC_F4,      KC_F5,   KC_F6,   KC_F7,      KC_F8,      KC_F9,      KC_F10,     KC_F11,     KC_F12,     DM_REC1,    KC_PSCR,    KC_SLCK,    KC_PAUS,    KC_NO,      KC_NO,      KC_MPRV,    U_VOLBRI},
                {   KC_GRV,     KC_1,       KC_2,       KC_3,    KC_4,       KC_5,    KC_6,    KC_7,       KC_8,       KC_9,       KC_0,       KC_MINS,    KC_EQL,     KC_BSPC,    KC_INS,     KC_HOME,    KC_PGUP,    KC_NLCK,    KC_PSLS,    KC_PAST,    KC_PMNS },
                {   KC_TAB,     KC_Q,       KC_W,       KC_E,    KC_R,       KC_T,    KC_Y,    KC_U,       KC_I,       KC_O,       KC_P,       KC_LBRC,    KC_RBRC,    KC_BSLS,    KC_DEL,     KC_END,     KC_PGDN,    KC_P7,      KC_P8,      KC_P9,      KC_PPLS },
                {   KC_CAPS,    KC_A,       KC_S,       KC_D,    KC_F,       KC_G,    KC_H,    KC_J,       KC_K,       KC_L,       KC_SCLN,    KC_QUOT,    KC_NO,      KC_ENT,     U_G5,       U_G4,       U_G3,       KC_P4,      KC_P5,      KC_P6,      KC_MNXT },
                {   KC_LSFT,    KC_NO,      KC_Z,       KC_X,    KC_C,       KC_V,    KC_B,    KC_N,       KC_M,       KC_COMM,    KC_DOT,     KC_SLSH,    KC_NO,      KC_RSFT,    U_G2,       KC_UP,      U_G1,       KC_P1,      KC_P2,      KC_P3,      KC_PENT },
                {   KC_LCTL,    KC_LGUI,    KC_LALT,    KC_SPC,  KC_RALT,    MO(_FN), KC_APP,  KC_B,       KC_RCTL,    KC_NO,      KC_NO,      KC_NO,      KC_NO,      KC_NO,      KC_LEFT,    KC_DOWN,    KC_RIGHT,   KC_H,       KC_P0,      KC_PDOT,    KC_MPLY }
              },
    [_FN]   = { {   RESET,      KC_MSEL,    KC_VOLD,    KC_VOLU, KC_MUTE,    KC_MSTP, KC_MPRV, KC_MPLY,    KC_MNXT,    KC_MAIL,    KC_WHOM,    KC_CALC,    DM_PLY1,    _______,    _______,    _______,    KC_SLEP,    _______,    _______,    _______,    _______ },
                {   _______,    _______,    _______,    _______, _______,    _______, _______, _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______ },
                {   _______,    _______,    _______,    _______, _______,    _______, _______, _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______ },
                {   _______,    _______,    _______,    _______, _______,    _______, _______, _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______ },
                {   _______,    _______,    _______,    _______, _______,    _______, _______, _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    RGB_VAI,    _______,    KC_BRIU,    KC_BRID,    _______,    _______ },
                {   _______,    _______,    _______,    _______, _______,    _______, _______, _______,    _______,    _______,    _______,    _______,    _______,    _______,    _______,    RGB_VAD,    _______,    _______,    _______,    _______,    _______ }
             }
};


/* Macro LED support
                    {G1, G2, G3, G4, G5, REC} */
int macro_state[] = {0,  0,  0,  0,  0,  0  };

// Map Macro state to RGB channels of LED indices 77 and 78
void set_macro_leds(int state[]) {
    rgb_matrix_set_color(118, state[2]*255, state[1]*255, state[0]*255);
    rgb_matrix_set_color(119, state[5]*255, state[4]*255, state[3]*255);
}

void macro_led_toggle(int i) {
    macro_state[i] = 1-macro_state[i];
    set_macro_leds(macro_state);
}

void macro_led_on(int i) {
    macro_state[i] = 1;
    set_macro_leds(macro_state);
}

void macro_led_off(int i) {
    macro_state[i] = 0;
    set_macro_leds(macro_state);
}

void macro_led_clear(void) {
    for (int i = 0; i < 6; i++){
        macro_state[i] = 0;
    }
    set_macro_leds(macro_state);
}

// Macro support (for default keymap)
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
    case U_VOLBRI:
        if (record->event.pressed) {
            // switch volume/brightness for encoder
            BRI = !BRI;
        } else {
            // on release
        }
        break;
    case U_G1:
        if (record->event.pressed) {
            macro_led_toggle(0);
        }
        break;
    case U_G2:
        if (record->event.pressed) {
            macro_led_toggle(1);
        }
        break;
    case U_G3:
        if (record->event.pressed) {
            macro_led_toggle(2);
        }
        break;
    case U_G4:
        if (record->event.pressed) {
            macro_led_toggle(3);
        }
        break;
    case U_G5:
        if (record->event.pressed) {
            macro_led_toggle(4);
        }
        break;
    case DM_REC1:
        macro_led_toggle(5);
        break;
    }
    return true;
};

//encoder support
void encoder_update_user(uint8_t index, bool clockwise) {
    if (index == 0) { /* First encoder */
        if (clockwise) {
            if(BRI){
                rgblight_increase_val_noeeprom();
            }else{
                tap_code(KC_VOLU);
            }
        } else {
            if(BRI){
                rgblight_decrease_val_noeeprom();
            }else{
                tap_code(KC_VOLD);
            }
        }
    }
}

// Customize Keyboard - tkon99
void keyboard_post_init_user(void) {
    //rgb_matrix_set_color_all(255,255,255);
    rgb_matrix_sethsv_noeeprom(0, 0, 200);
}
