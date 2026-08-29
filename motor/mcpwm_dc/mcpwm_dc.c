/**
 * This file contains all high-level logic, so nothing to do with the hardware.
 * However, control loops and callbacks are not handled here.
 */

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mcpwm_dc.h"
#include "mc_interface.h"
#include "digital_filter.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "ledpwm.h"
#include "terminal.h"
#include "timeout.h"
#include "encoder/encoder.h"
#include "timer.h"

#include "mcpwm_dc_locals.h"
#include "mcpwm_dc_hw.h"
#include "mcpwm_dc.h"

void mcpwm_dc_init(volatile mc_configuration *configuration)
{
    utils_sys_lock_cnt();

    init_done = false;

    conf = configuration;

    mcpwm_dc_init_locals();

    mcpwm_dc_init_hw();

    // Check if the system has resumed from IWDG reset
    if (timeout_had_IWDG_reset())
    {
        mc_interface_fault_stop(FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET, false, false);
    }
    
    init_done = true;
}

void mcpwm_dc_deinit(void)
{
    if (!init_done)
    {
        return;
    }

    init_done = false;

    mcpwm_dc_deinit_hw();
}

void mcpwm_dc_set_configuration(volatile mc_configuration *configuration) {
	// Stop everything first to be safe
	control_mode = CONTROL_MODE_NONE;
	//stop_pwm_ll();
    // TODO: should be ll, think about integrating that
    stop_pwm_hw();

	utils_sys_lock_cnt();
	conf = configuration;
	utils_sys_unlock_cnt();
}

