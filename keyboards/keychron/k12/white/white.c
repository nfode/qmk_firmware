/* Copyright 2021 IsaacDynamo
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
#include "white.h"

// Manage Windows and Mac LEDs
// - Show status of mode switch
// - Turn LEDs off durring USB suspend
static bool mode_leds_show = true;
static bool mode_leds_windows;

static void mode_leds_update(void){
    writePin(LED_WIN_PIN, mode_leds_show && mode_leds_windows);
    writePin(LED_MAC_PIN, mode_leds_show && !mode_leds_windows);
}

bool dip_switch_update_kb(uint8_t index, bool active){
    if(index == 0) {
        if(active) { // Mac mode
            layer_move(MAC_BASE);
        } else { // Windows mode
            layer_move(WIN_BASE);
        }

        // Update mode and update leds
        mode_leds_windows = !active;
        mode_leds_update();
    }

    dip_switch_update_user(index, active);
    return true;
}

void keyboard_pre_init_kb(void) {
    // Setup Win & Mac LED Pins as output
    setPinOutput(LED_WIN_PIN);
    setPinOutput(LED_MAC_PIN);

    // WORKAROUND: Mac & Windows LED flicker.
    // Normally the GPIOs in DIP_SWITCH_PINS will be initialized by dip_switch_init().
    // But during startup of the keyboard the Mac/Windows dip switch seems to jitter, causing the Mac and Windows LEDs to flicker.
    // Maybe the internal pull-up of this chip is really weak, and needs some time to pullup the input voltage to the high level? Seems unlikely but cannot think of a better explanation.
    // By doing the configuration of the GPIOs here the pullup is enabled earlier and the flicker is gone.
    const pin_t dip_switch_pad[] = DIP_SWITCH_PINS;
    const size_t size = sizeof(dip_switch_pad) / sizeof(dip_switch_pad[0]);
    for (size_t i=0; i<size; i++) {
        setPinInputHigh(dip_switch_pad[i]);
    }

    keyboard_pre_init_user();
}

void suspend_power_down_kb(void) {
    // Turn leds off
    mode_leds_show = false;
    mode_leds_update();


    #ifdef RGB_MATRIX_ENABLE
    // Suspend backlight
    rgb_matrix_set_suspend_state(true);
    #endif

    suspend_power_down_user();
}

/// TODO: Clean-up workaround
/// Currently the suspend_wakeup_init_kb() has issues. See https://github.com/SonixQMK/qmk_firmware/issues/80
/// A workaround is to use housekeeping_task_kb() instead.
void housekeeping_task_kb(void) {
    // Turn on
    mode_leds_show = true;
    mode_leds_update();

    #ifdef RGB_MATRIX_ENABLE
    // Restore backlight
    rgb_matrix_set_suspend_state(false);
    #endif

    housekeeping_task_user();
}

#ifdef RGB_MATRIX_ENABLE

#include "rgb_matrix.h"

#define NA NO_LED

// Mark keys that are black & orange with the default keychron keycaps.
// This is used by the custom rgb matrix effect to create a high contrast mode that only lights up black keys or black and orange keys. To make them better readable.
#define B (128 | 4) // Black
#define O (64  | 4) // Orange, Esc and lightbulb are assumed to be orange

/// Force g_led_config into flash, because there is no space in RAM.
/// This should be safe because g_led_config should never be written to.
/// We cannot make g_led_config const, because rgb_matrix.h, exports it as mutable.
__attribute__(( section(".rodata.g_led_config") ))
led_config_t g_led_config = { {
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13 },
    { 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 },
    { 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, NA, 40 },
    { 41, NA, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, NA, 52 },
    { 53, 54, 55, NA, NA, NA, 56, NA, NA, NA, 57, 58, 59, 60 }
}, {
    {0  ,  0}, {12 ,  0}, {25 ,  0}, {37 ,  0}, {50 ,  0}, {62 ,  0}, {75 ,  0}, {87 ,  0}, {100,  0}, {112,  0}, {124,  0}, {137,  0}, {149,  0}, {168,  0},
    {3  , 13}, {19 , 13}, {31 , 13}, {44 , 13}, {56 , 13}, {68 , 13}, {81 , 13}, {93 , 13}, {106, 13}, {118, 13}, {131, 13}, {143, 13}, {155, 13}, {168, 13},
    {5  , 26}, {22 , 26}, {34 , 26}, {47 , 26}, {59 , 26}, {72 , 26}, {84 , 26}, {96 , 26}, {109, 26}, {121, 26}, {134, 26}, {146, 26},            {166, 26},
    {8  , 38},            {28 , 38}, {40 , 38}, {53 , 38}, {65 , 38}, {78 , 38}, {90 , 38}, {103, 38}, {115, 38}, {128, 38}, {140, 38},            {165, 38},
    {2  , 51}, {17 , 51}, {33 , 51},                                  {79 , 51},                                  {126, 51}, {141, 51}, {156, 51}, {171, 51},
}, {
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,    4,
    4,    4, 4, 4, 4, 4, 4, 4, 4, 4, 4,    4,
    4, 4, 4,          4,          4, 4, 4, 4,
} };

#endif
