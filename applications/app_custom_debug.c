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

    Custom app to debug the internal state of the brushed dc motor module
    */

#ifdef MCPWM_DC_DEBUG
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
#include "motor/mc_interface.h"
#include "motor/mcpwm_dc/mcpwm_dc.h"
#include "motor/mcpwm_dc/mcpwm_dc_hw.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Private functions
static void terminal_debug(int argc, const char **argv);

/**
 * Callback function for the terminal. Used to debug the internal state of the parking brake
 */
static void terminal_debug(int argc, const char **argv)
{
    // Get output
    const volatile mc_configuration *conf = mc_interface_get_configuration();
    mc_state state = mc_interface_get_state();
    mc_control_mode control_mode = mc_interface_get_control_mode();
    bool h_b_c = mcpwm_dc_was_h_bridge_configured();
    float duty_cycle_set = mc_interface_get_duty_cycle_set();
    float duty_cycle_now = mc_interface_get_duty_cycle_now();
    float tot_current = mc_interface_get_tot_current();
    float tot_current_filtered = mc_interface_get_tot_current_filtered();
    float tot_current_directional = mc_interface_get_tot_current_directional();
    float tot_current_directional_filtered = mc_interface_get_tot_current_directional_filtered();
    float tot_current_in = mc_interface_get_tot_current_in();
    float tot_current_in_filtered = mc_interface_get_tot_current_in_filtered();
    float switching_frequency_now = mcpwm_dc_get_switching_frequency_now();
    bool dccal_done = mc_interface_dccal_done();
    bool init_done = mcpwm_dc_init_done();
    mc_timer_struct timer_struct = mcpwm_dc_hw_get_timer_config();

    float curr_pb = mcpwm_dc_get_tot_pb_current();
    float curr_pb_filt = mcpwm_dc_get_tot_pb_current_filtered();
    mc_control_mode state_pb = mcpwm_dc_get_state_parking_brake();
    bool is_parking_brake_engaged = mcpwm_dc_is_parking_brake_engaged();

    int curr_adc_source_mask, curr_start_samples, curr0_sum, curr1_sum, curr2_sum, curr0_offset, curr1_offset, curr2_offset;
    mcpwm_dc_meas_get_info(&curr_adc_source_mask, &curr_start_samples, &curr0_sum, &curr1_sum, &curr2_sum, &curr0_offset, &curr1_offset, &curr2_offset);

    // Print it
    commands_printf("DEBUG:");
    commands_printf("  Motor type: %d", conf->motor_type);
    commands_printf("  State: %d", state);
    commands_printf("  Control mode: %d", control_mode);
    commands_printf("  H bridge: %d", h_b_c ? 1 : 0);
    commands_printf("  duty set: %.3f", (double)duty_cycle_set);
    commands_printf("  duty now: %.3f", (double)duty_cycle_now);
    commands_printf("  tot_current: %.3f", (double)tot_current);
    commands_printf("  tot_current_filtered: %.3f", (double)tot_current_filtered);
    commands_printf("  tot_current_directional: %.3f", (double)tot_current_directional);
    commands_printf("  tot_current_directional_filtered: %.3f", (double)tot_current_directional_filtered);
    commands_printf("  tot_current_in: %.3f", (double)tot_current_in);
    commands_printf("  tot_current_in_filtered: %.3f", (double)tot_current_in_filtered);
    commands_printf("  switching_frequency_now: %.3f", (double)switching_frequency_now);
    commands_printf("  init_done: %d", init_done ? 1 : 0);
    commands_printf("  dccal_done: %d", dccal_done ? 1 : 0);
    commands_printf("  Latest ADC duration: %.4f ms", (double)(mc_interface_get_last_adc_isr_duration() * 1000.0));
    commands_printf("  Latest injected ADC duration: %.4f ms", (double)(mc_interface_get_last_inj_adc_isr_duration() * 1000.0));
    commands_printf("  Timer:");
    commands_printf("    Top: %u", timer_struct.top);
    commands_printf("    Duty motor: %u", timer_struct.duty_motor);
    commands_printf("    Duty brake: %u", timer_struct.duty_brake);
    commands_printf("    val_sample: %u", timer_struct.val_sample);
    commands_printf("    curr1_sample: %u", timer_struct.curr1_sample);
    commands_printf("    curr2_sample: %u", timer_struct.curr2_sample);
    commands_printf("    curr3_sample: %u", timer_struct.curr3_sample);
    commands_printf("  Measurement:");
    commands_printf("    curr_adc_source_mask: %d", curr_adc_source_mask);
    commands_printf("    curr_start_samples: %d", curr_start_samples);
    commands_printf("    curr0_sum: %d", curr0_sum);
    commands_printf("    curr1_sum: %d", curr1_sum);
    commands_printf("    curr2_sum: %d", curr2_sum);
    commands_printf("    curr0_offset: %d", curr0_offset);
    commands_printf("    curr1_offset: %d", curr1_offset);
    commands_printf("    curr2_offset: %d", curr2_offset);
    commands_printf("  Parking brake:");
    commands_printf("    Current: %.3f A", (double)curr_pb);
    commands_printf("    Current filtered: %.3f A", (double)curr_pb_filt);
    commands_printf("    State: %d", state_pb);
    commands_printf("    Engaged: %d", is_parking_brake_engaged ? 1 : 0);
}
#endif

/**
 * Called when the application is started
 * Set a callback
 */
void app_custom_debug_start(void)
{
#ifdef MCPWM_DC_DEBUG
    // Terminal commands for the VESC Tool terminal can be registered.
    terminal_register_command_callback(
        "debug",
        "Print debug info about mcpwm module",
        NULL,
        terminal_debug);
#endif
}

/**
 * Called when the application is stopped
 * Release callback.
 */
void app_custom_debug_stop(void)
{
#ifdef MCPWM_DC_DEBUG
    terminal_unregister_callback(terminal_debug);
#endif
}