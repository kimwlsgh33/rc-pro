/**
 * Stepper Motor Driver
 *
 * used by precise Peristaltic Pump, Syringe
 */
#ifndef __SM1_H__
#define __SM1_H__

#include "../../config/cfg_build.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"
#include "../../core/module_interface.h"

/*
        Run
*/
int sm1_init();

int sm1_get_state();

int sm1_run(int dir, int speedhz);

int sm1_move_to(int32_t targetPos);

void sm1_stop();

/**
 * current position in pulse count
 * org is zero
 */
int32 sm1_get_position();

/*
        Set current position as initial position, zero
*/
int sm1_reset_position();

void sm1_enable();

void sm1_disable();

void sm1_process();

#endif
//__SM1_H__
