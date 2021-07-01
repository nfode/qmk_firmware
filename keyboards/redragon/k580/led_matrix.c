#include <string.h>
#include "rgb.h"
#include "rgb_matrix.h"
#include "rgb_matrix_types.h"
#include "color.h"

/*
    COLS key / led
    PWM PWM00A - PWM21A (PWM15A unused)
    2ty transistors PNP driven high
    base      - GPIO
    collector - LED Col pins
    emitter   - VDD

    VDD     GPIO
    (E)     (B)
     |  PNP  |
     |_______|
         |
         |
        (C)
        LED

    ROWS RGB
    PWM PWM22A - PWM21B (PWM10B unused)
    C 0-15
    j3y transistors NPN driven low
    base      - GPIO
    collector - LED RGB row pins
    emitter   - GND

        LED
        (C)
         |
         |
      _______
     |  NPN  |
     |       |
    (B)     (E)
    GPIO    GND
*/

LED_TYPE led_state[LED_MATRIX_ROWS * LED_MATRIX_COLS];
uint8_t led_pos[DRIVER_LED_TOTAL + 2];
#ifdef UNDERGLOW_IDX
static const unsigned int underglow_led_idx[UNDERGLOW_LEDS] = UNDERGLOW_IDX;
#endif


void init(void) {
    unsigned int i = 0;
    for (unsigned int y = 0; y < LED_MATRIX_ROWS; y++) {
        for (unsigned int x = 0; x < LED_MATRIX_COLS; x++) {
            if (g_led_config.matrix_co[y][x] != NO_LED) {
                led_pos[g_led_config.matrix_co[y][x]] = i;
            }
            i++;
        }
    }
    led_pos[DRIVER_LED_TOTAL] = 77;
    led_pos[DRIVER_LED_TOTAL + 1] = 78;
#ifdef UNDERGLOW_IDX
    i = 0;
    for (unsigned int x = DRIVER_LED_TOTAL - UNDERGLOW_LEDS; x < DRIVER_LED_TOTAL; x++) {
        led_pos[x] = underglow_led_idx[i];
        i++;
    }
#endif
}

static void flush(void) {}

void set_color(int index, uint8_t r, uint8_t g, uint8_t b) {
    int corrected_index = led_pos[index];
    for (unsigned int i = 0; i < UNDERGLOW_LEDS; i++) {
        if (corrected_index == underglow_led_idx[i]) {
            led_state[corrected_index].r = r;
            led_state[corrected_index].g = b;
            led_state[corrected_index].b = g;
            return;
        }
    }
    led_state[corrected_index].r = r;
    led_state[corrected_index].g = g;
    led_state[corrected_index].b = b;
}

static void set_color_all(uint8_t r, uint8_t g, uint8_t b) {
    for (int i=0; i<DRIVER_LED_TOTAL; i++)
        set_color(i, r, g, b);
}

const rgb_matrix_driver_t rgb_matrix_driver = {
    .init          = init,
    .flush         = flush,
    .set_color     = set_color,
    .set_color_all = set_color_all,
};

void led_set(uint8_t usb_led) {
    writePin(LED_NUM_LOCK_PIN, !IS_LED_ON(usb_led, USB_LED_NUM_LOCK));
    writePin(LED_CAPS_LOCK_PIN, !IS_LED_ON(usb_led, USB_LED_CAPS_LOCK));
    writePin(LED_SCROLL_LOCK_PIN, !IS_LED_ON(usb_led, USB_LED_SCROLL_LOCK));
}
