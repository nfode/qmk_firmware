#include "rgb_matrix.h"
#include "rgb_matrix_sn32f24xx.h"

/*
    COLS key / led
    Y2 transistors PNP driven high
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
    Y1 transistors NPN driven low
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
static uint16_t row_ofsts[LED_MATRIX_ROWS];
static uint8_t mr_offset[24] = {0};
static uint8_t current_row = 0;
LED_TYPE led_state[LED_MATRIX_ROWS * LED_MATRIX_COLS];
uint8_t led_pos[DRIVER_LED_TOTAL];
extern matrix_row_t raw_matrix[MATRIX_ROWS]; //raw values
static const pin_t led_row_pins[LED_MATRIX_ROWS_HW] = LED_MATRIX_ROW_PINS;
static const pin_t led_col_pins[LED_MATRIX_COLS] = LED_MATRIX_COL_PINS;

/* PWM configuration structure. We use timer CT16B1 with 24 channels. */
static PWMConfig pwmcfg = {
    675250,        /* PWM clock frequency. */
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
                cfg->channels[0].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[0] = i;
                break;

            case B9:
                cfg->channels[1].pfpamsk = 1;
            case A1:
                cfg->channels[1].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[1] = i;
                break;
            
            case B10:
                cfg->channels[2].pfpamsk = 1;
            case A2:
                cfg->channels[2].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[2] = i;
                break;

            case B11:
                cfg->channels[3].pfpamsk = 1;
            case A3:
                cfg->channels[3].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[3] = i;
                break;

            case B12:
                cfg->channels[4].pfpamsk = 1;
            case A4:
                cfg->channels[4].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[4] = i;
                break;

            case B13:
                cfg->channels[5].pfpamsk = 1;
            case A5:
                cfg->channels[5].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[5] = i;
                break;

            case B14:
                cfg->channels[6].pfpamsk = 1;
            case A6:
                cfg->channels[6].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[6] = i;
                break;

            case B15:
                cfg->channels[7].pfpamsk = 1;
            case A7:
                cfg->channels[7].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[7] = i;
                break;

            case C0:
                cfg->channels[8].pfpamsk = 1;
            case A8:
                cfg->channels[8].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[8] = i;
                break;

            case C1:
                cfg->channels[9].pfpamsk = 1;
            case A9:
                cfg->channels[9].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[9] = i;
                break;

            case C2:
                cfg->channels[10].pfpamsk = 1;
            case A10:
                cfg->channels[10].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[10] = i;
                break;

            case C3:
                cfg->channels[11].pfpamsk = 1;
            case A11:
                cfg->channels[11].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[11] = i;
                break;

            case C4:
                cfg->channels[12].pfpamsk = 1;
            case A12:
                cfg->channels[12].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[12] = i;
                break;

            case C5:
                cfg->channels[13].pfpamsk = 1;
            case A13:
                cfg->channels[13].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[13] = i;
                break;

            case C6:
                cfg->channels[14].pfpamsk = 1;
            case A14:
                cfg->channels[14].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[14] = i;
                break;

            case C7:
                cfg->channels[15].pfpamsk = 1;
            case A15:
                cfg->channels[15].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[15] = i;
                break;

            case C8:
                cfg->channels[16].pfpamsk = 1;
            case B0:
                cfg->channels[16].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[16] = i;
                break;

            case C9:
                cfg->channels[17].pfpamsk = 1;
            case B1:
                cfg->channels[17].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[17] = i;
                break;

            case C10:
                cfg->channels[18].pfpamsk = 1;
            case B2:
                cfg->channels[18].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[18] = i;
                break;

            case C11:
                cfg->channels[19].pfpamsk = 1;
            case B3:
                cfg->channels[19].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[19] = i;
                break;

            case C12:
                cfg->channels[20].pfpamsk = 1;
            case B4:
                cfg->channels[20].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[20] = i;
                break;

            case C13:
                cfg->channels[21].pfpamsk = 1;
            case B5:
                cfg->channels[21].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[21] = i;
                break;

            case C14:
                cfg->channels[22].pfpamsk = 1;
            case B6:
                cfg->channels[22].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[22] = i;
                break;

            case C15:
                cfg->channels[23].pfpamsk = 1;
            case B7:
                cfg->channels[23].mode = PWM_OUTPUT_ACTIVE_LOW;
                mr_offset[23] = i;
                break;
        }
    }

}
void rgb_callback(PWMDriver *pwmp);

void shared_matrix_rgb_enable(void) {
    pwmcfg.callback = rgb_callback;
    pwmEnablePeriodicNotification(&PWMD1);
}

void shared_matrix_rgb_disable(void) {
    // Disable PWM outputs on column pins
    for(uint8_t i=0;i<24;i++){
        chSysLockFromISR();
        pwmDisableChannel(&PWMD1,i);
        chSysUnlockFromISR();
    }
    pwmcfg.callback = NULL;
    pwmDisablePeriodicNotification(&PWMD1);
}

void rgb_callback(PWMDriver *pwmp) {

    // Disable the interrupt
    shared_matrix_rgb_disable();
    // Turn the selected row off
    writePinLow(led_row_pins[current_row]);

    // Turn the next row on
    current_row = (current_row + 1) % LED_MATRIX_ROWS_HW;

    matrix_scan_keys(raw_matrix,current_row);

    uint8_t row_idx = ( current_row / 3 );
    uint16_t row_ofst = row_ofsts[row_idx];
    
    for(uint8_t i=0; i<24; i++){
        uint8_t state_r = led_state[row_ofst + mr_offset[i] ].r;
        uint8_t state_b = led_state[row_ofst + mr_offset[i] ].b;
        uint8_t state_g = led_state[row_ofst + mr_offset[i] ].g;
        switch(current_row % 3) {
        case 0:
            if(state_r >0) {
                chSysLockFromISR();
                pwmEnableChannelI(pwmp,i,state_r);
                chSysUnlockFromISR();
                break;
            }
            goto killit;
        case 1:
            if(state_b >0) {
                chSysLockFromISR();
                pwmEnableChannelI(pwmp,i,state_b);
                chSysUnlockFromISR();
                break;
            }
            goto killit;
        case 2:
            if(state_g >0) {
                chSysLockFromISR();
                pwmEnableChannelI(pwmp,i,state_g);
                chSysUnlockFromISR();
                break;
            }
            goto killit;
        default:
        killit:
            chSysLockFromISR();
            pwmDisableChannelI(pwmp,i);
            chSysUnlockFromISR();
        }
    }
    // Enable the interrupt
    shared_matrix_rgb_enable();
    //Restart with new values
    pwmStart(pwmp, &pwmcfg);
    writePinHigh(led_row_pins[current_row]);
}

void SN32F24XX_init(void) {
    unsigned int i = 0;
    for (unsigned int y = 0; y < LED_MATRIX_ROWS; y++) {
        for (unsigned int x = 0; x < LED_MATRIX_COLS; x++) {
            if (g_led_config.matrix_co[y][x] != NO_LED) {
                led_pos[g_led_config.matrix_co[y][x]] = i;
            }
            i++;
        }
    }
    for (uint8_t x = 0; x < LED_MATRIX_ROWS_HW; x++) {
        setPinOutput(led_row_pins[x]);
        writePinHigh(led_row_pins[x]);
    }
    for (uint8_t i = 0; i < LED_MATRIX_ROWS; i++) {
        row_ofsts[i] = i * LED_MATRIX_COLS;
    }

    rgb_ch_ctrl(&pwmcfg);
    pwmStart(&PWMD1, &pwmcfg);
    shared_matrix_rgb_enable();
}

static void flush(void) {}

void SN32F24XX_set_color(int index, uint8_t r, uint8_t g, uint8_t b) {
    int corrected_index = led_pos[index];
    led_state[corrected_index].r = r;
    led_state[corrected_index].g = g;
    led_state[corrected_index].b = b;
}

void SN32F24XX_set_color_all(uint8_t r, uint8_t g, uint8_t b) {
    for (int i=0; i<DRIVER_LED_TOTAL; i++)
        SN32F24XX_set_color(i, r, g, b);
}

const rgb_matrix_driver_t rgb_matrix_driver = {
    .init          = SN32F24XX_init,
    .flush         = flush,
    .set_color     = SN32F24XX_set_color,
    .set_color_all = SN32F24XX_set_color_all,
};