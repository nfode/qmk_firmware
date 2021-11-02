#pragma once

#include <stdint.h>
#include <stdbool.h>

void init(void);
void set_color(int index, uint8_t r, uint8_t g, uint8_t b);
void set_color_all(uint8_t r, uint8_t g, uint8_t b);
void shared_matrix_enable(void);
void shared_matrix_disable(void);
void rgb_callback(PWMDriver *pwmp);