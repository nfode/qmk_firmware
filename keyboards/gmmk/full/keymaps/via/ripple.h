#ifndef DISABLE_RGB_MATRIX_RIPPLE
#    define RGB_MATRIX_RIPPLE
RGB_MATRIX_EFFECT(RIPPLE)
#    ifdef RGB_MATRIX_CUSTOM_EFFECT_IMPLS

#        define RIPPLE_MAX_NUM 3
#        define RIPPLE_SPEED_MAX 5
#        define RIPPLE_X_MAX 224
#        define RIPPLE_Y_MAX 64
#        define RIPPLE_R_MAX 232
#        define RIPPLE_MIN_HSV_V 10
#        define RIPPLE_SPEED 16
#        define RIPPLE_DELAY 100

#        define ABS(x) ((x) > 0 ? (x) : -(x))
#        define RAND(x) ((uint32_t)rand() % (uint32_t)(x))

typedef struct {
    uint8_t  cx, cy;
    uint8_t  delay;
    uint8_t  speed;    
    uint16_t r;
    uint16_t thick;
    HSV      hsv;
} ripple_t;

static ripple_t ripples[RIPPLE_MAX_NUM];

static bool is_on_ripple(int32_t x, int32_t y, int32_t cx, int32_t cy, int32_t r, int32_t thick) {
    int32_t dx, dy, r2, s;

    dx = x - cx;
    dy = y - cy;
    r >>= 1;
    r2 = r * r;
    s  = dx * dx + dy * dy - r2;
    s  = ABS(s);

    if (s <= thick)
        return true;
    else
        return false;
}

static void init_ripple(ripple_t *rp) {
    rp->cx    = RAND(RIPPLE_X_MAX);
    rp->cy    = RAND(RIPPLE_Y_MAX);
    rp->r     = RAND(10) + 2;
    rp->delay = RAND(scale8(RIPPLE_DELAY, rgb_matrix_config.speed) + 1) + 40;
    rp->speed = RAND(scale8(RIPPLE_SPEED, rgb_matrix_config.speed) + 1) + 2;
    rp->hsv.h = RAND(255);
    rp->hsv.s = 150 + RAND(100);
    rp->hsv.v = rgb_matrix_config.hsv.v;
    rp->thick = RAND(1500) + 700;
}

bool RIPPLE(effect_params_t *params) {
    RGB_MATRIX_USE_LIMITS(led_min, led_max);

    if (params->init) {
        for (int32_t i = 0; i < RIPPLE_MAX_NUM; i++) {
            ripple_t *rp = ripples + i;
            init_ripple(rp);
        }
    }

    if (led_min == 0) {
        for (int32_t j = 0; j < RIPPLE_MAX_NUM; j++) {
            ripple_t *rp = ripples + j;

            if (rp->delay)
                rp->delay--;
            else {
                int32_t r = rp->r / 2;

                if (r > RIPPLE_R_MAX) {
                    init_ripple(rp);
                } else {
                    rp->r += rp->speed;

                    // degrade V with radius
                    int32_t new_v = (int32_t)rp->hsv.v - (int32_t)(rp->speed / 2);

                    if (new_v < RIPPLE_MIN_HSV_V)
                        rp->hsv.v = RIPPLE_MIN_HSV_V;
                    else
                        rp->hsv.v = (uint32_t)new_v;
                }
            }
        }
    }

    for (uint8_t i = led_min; i < led_max; i++) {
        RGB_MATRIX_TEST_LED_FLAGS();

        bool     is_rendered = false;
        uint32_t r = 0, g = 0, b = 0;

        for (int32_t j = 0; j < RIPPLE_MAX_NUM; j++) {
            ripple_t *rp        = ripples + j;
            bool      is_ripple = (rp->delay == 0) && is_on_ripple(g_led_config.point[i].x, g_led_config.point[i].y, rp->cx, rp->cy, rp->r, rp->thick);
            if (is_ripple) {
                RGB rgb = rgb_matrix_hsv_to_rgb(rp->hsv);
                r += rgb.r;
                g += rgb.g;
                b += rgb.b;
                is_rendered = true;
            }
        }

        if (is_rendered) {
            if (r > 255) r = 255;
            if (g > 255) g = 255;
            if (b > 255) b = 255;
            rgb_matrix_set_color(i, r, g, b);
        } else
            rgb_matrix_set_color(i, 0, 0, 0);
    }

    return led_max < DRIVER_LED_TOTAL;
}

#    endif  // RGB_MATRIX_CUSTOM_EFFECT_IMPLS
#endif      // DISABLE_RGB_MATRIX_BREATHING
