#pragma once

#include "quantum.h"

// Each layer gets a name for readability
enum layer_names {
    WIN_BASE    = 0,
    WIN_FN      = 1,
    MAC_BASE    = 2,
    MAC_FN      = 3,
    FN2      = 4
};

// readability
#define XXX KC_NO

#define LAYOUT( \
    k00, k01, k02, k03, k04, k05, k06, k07, k08, k09, k0A, k0B, k0C, k0D, \
    k10, k11, k12, k13, k14, k15, k16, k17, k18, k19, k1A, k1B, k1C, k1D, \
    k20, k21, k22, k23, k24, k25, k26, k27, k28, k29, k2A, k2B,      k2D, \
    k30,      k32, k33, k34, k35, k36, k37, k38, k39, k3A, k3B,      k3D, \
    k40, k41, k42,                k46,                k4A, k4B, k4C, k4D  \
) { \
    { k00, k01, k02, k03, k04, k05, k06, k07, k08, k09, k0A, k0B, k0C, k0D }, \
    { k10, k11, k12, k13, k14, k15, k16, k17, k18, k19, k1A, k1B, k1C, k1D }, \
    { k20, k21, k22, k23, k24, k25, k26, k27, k28, k29, k2A, k2B, XXX, k2D }, \
    { k30, XXX, k32, k33, k34, k35, k36, k37, k38, k39, k3A, k3B, XXX, k3D }, \
    { k40, k41, k42, XXX, XXX, XXX, k46, XXX, XXX, XXX, k4A, k4B, k4C, k4D }, \
}
