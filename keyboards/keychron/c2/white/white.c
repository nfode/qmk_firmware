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
/*
static bool mode_leds_show = true;
static bool mode_leds_windows;
*/

//static void mode_leds_update(void) {
    //writePin(LED_WIN_PIN, mode_leds_show && mode_leds_windows);
    //writePin(LED_MAC_PIN, mode_leds_show && !mode_leds_windows);
//}

//void dip_switch_update_kb(uint8_t index, bool active){
//    if(index == 0) {
//        if(active) { // Windows mode
//            layer_move(WIN_BASE);
//        } else { // Mac mode
//            layer_move(WIN_BASE);
//        }
//
//        // Update mode and update leds
//        mode_leds_windows = active;
//        mode_leds_update();
//    }
//
//    dip_switch_update_user(index, active);
//}

void keyboard_pre_init_kb(void) {
    // Setup Win & Mac LED Pins as output
   // setPinOutput(LED_WIN_PIN);
   // setPinOutput(LED_MAC_PIN);
   // writePinLow(LED_WIN_PIN);
   // writePinLow(LED_MAC_PIN);

   // // Set status leds pins
   setPinOutput(LED_CAPS_LOCK_PIN);

    keyboard_pre_init_user();
}

void suspend_power_down_kb(void) {
    // Turn leds off
    //mode_leds_show = false;
    //mode_leds_update();

    // Suspend backlight
    //rgb_matrix_set_suspend_state(true);

    suspend_power_down_user();
}

/// TODO: Clean-up workaround
/// Currently the suspend_wakeup_init_kb() has issues. See https://github.com/SonixQMK/qmk_firmware/issues/80
/// A workaround is to use housekeeping_task_kb() instead.
void housekeeping_task_kb(void) {
    // Turn on
   // mode_leds_show = true;
    // mode_leds_update();

    // Restore backlight
    //rgb_matrix_set_suspend_state(false);

    housekeeping_task_user();
}

#if CH_CFG_NO_IDLE_THREAD == TRUE

#    define CYCLES_PER_LOOP 9
#    define LOOP_TIMES (48000000 / (CH_CFG_ST_FREQUENCY) / (CYCLES_PER_LOOP))

void chThdSleep(sysinterval_t time) {
    uint32_t loops = time * LOOP_TIMES;

    for (uint32_t i = 0; i < loops; i++) __NOP();
}

/* suspend thread used in usb_main.c */
msg_t chThdSuspendTimeoutS(thread_reference_t* trp, sysinterval_t timeout) {
    osalSysUnlock();
    osalSysLock();

    return MSG_OK;
}

#endif /* CH_CFG_NO_IDLE_THREAD */


#ifdef RGB_MATRIX_ENABLE

#include "rgb_matrix.h"

#define XX NO_LED

// Mark keys that are black with the default keychron keycaps.
// This is used but the custom rgb matrix effect to create a high contrast mode that only lights up black keys. To make them better readable.
#define B (128 | 4)

/// Force g_led_config into flash, because there is no space in RAM.
/// This should be safe because g_led_config should never be written to.
/// We cannot make g_led_config const, because rgb_matrix.h, exports it as mutable.
__attribute__(( section(".rodata.g_led_config") ))
led_config_t g_led_config = {
{
  {  16,  17,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27  },
  {  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48  },
  {  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69  },
  {  74,  XX,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84  },
  {  91,  92,  93,  XX,  XX,  94,  XX,  XX,  XX,  XX,  95,  96   }
}, {
    { 1 *12, 13 }, { 2 *12, 13 }, { 3 *12, 13 }, { 4 *12, 13 }, { 5 *12, 13 }, { 6 *12, 13 }, { 7 *12, 13 }, { 8 *12, 13 }, { 9 *12, 13 }, { 10*12, 13 }, 
    { 1 *12, 26 }, { 2 *12, 26 }, { 3 *12, 26 }, { 4 *12, 26 }, { 5 *12, 26 }, { 6 *12, 26 }, { 7 *12, 26 }, { 8 *12, 26 }, { 9 *12, 26 }, { 10*12, 26 }, 
    { 1 *12, 38 }, { 2 *12, 38 }, { 3 *12, 38 }, { 4 *12, 38 }, { 5 *12, 38 }, { 6 *12, 38 }, { 7 *12, 38 }, { 8 *12, 38 }, { 9 *12, 38 }, { 10*12, 38 }, 
    { 1 *12, 51 },                { 3 *12, 51 }, { 4 *12, 51 }, { 5 *12, 51 }, { 6 *12, 51 }, { 7 *12, 51 }, { 8 *12, 51 }, { 9 *12, 51 }, { 10*12, 51 },
    { 1 *12, 64 }, { 2 *12, 64 }, { 3 *12, 64 },                               { 6 *12, 64 },                               { 9*12, 64 }, { 10*12 ,64}
}, {
   B, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, B,
   B, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, B,
   B, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,    B,
   B,    4, 4, 4, 4, 4, 4, 4, 4, 4, 4,    B,
   B, B, B,       4,             B, B, B, B
}
};

#endif
