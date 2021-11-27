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
#include "noled.h"

#define DEBUG_MATRIX_SCAN_RATE

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