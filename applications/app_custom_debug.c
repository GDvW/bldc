/*
    Copyright 2019 Benjamin Vedder	benjamin@vedder.se

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Custom app to debug the internal state of the parking brake
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
#include "mcpwm_dc.h"
#include "mcpwm_dc_parking_brake.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Private functions
static void terminal_debug(int argc, const char **argv);

/**
 * Called when the application is started
 * Set a callback
 */
void app_custom_start(void)
{
    // Terminal commands for the VESC Tool terminal can be registered.
    terminal_register_command_callback(
        "print_debug_parking",
        "Print debug info about parking",
        "[d]",
        terminal_debug);
}

/**
 * Called when the application is stopped
 * Release callback.
 */
void app_custom_stop(void)
{
    terminal_unregister_callback(terminal_debug);
}

/**
 * Set the app configuration.
 *
 * @param conf the app configuration
 */
void app_custom_configure(app_configuration *conf)
{
    (void)conf;
}

/**
 * Callback function for the terminal. Used to debug the internal state of the parking brake
 */
static void terminal_debug(int argc, const char **argv)
{

    // Get output
    bool is_enabled = mcpwm_dc_is_parking_brake_enabled();
    bool is_engaged = mcpwm_dc_is_parking_brake_engaged();
    float duty = mcpwm_dc_get_parking_brake_duty();
    uint16_t ccr_call_n = mcpwm_dc_get_ccr_called();
    uint32_t reg_CCR2 = mcpwm_reg_get_CCR2();
    uint32_t reg_ARR = mcpwm_reg_get_ARR();
    uint32_t reg_CCER = mcpwm_reg_get_CCER();
    uint32_t reg_CCMR1 = mcpwm_reg_get_CCMR1();
    uint32_t reg_BDTR = mcpwm_reg_get_BDTR();

    // Print it
    commands_printf("PB enabled=%d engaged=%d duty=%.3f called=%d",
                    is_enabled, is_engaged, (double)duty, ccr_call_n);
    commands_printf("	reg_CCR2=%d", reg_CCR2);
    commands_printf("	reg_ARR=%d", reg_ARR);
    commands_printf("	reg_CCER=%d", reg_CCER);
    commands_printf("	reg_CCMR1=%d", reg_CCMR1);
    commands_printf("	reg_BDTR=%d", reg_BDTR);
}
