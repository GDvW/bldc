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

#define MCPWM_DC_DEBUG
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
static void terminal_reset_bools(int argc, const char **argv);

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

    float duty_now_parking_brake = mcpwm_dc_get_duty_parking_brake();
    bool was_parking_h_bridge_updated = mcpwm_dc_was_parking_h_bridge_updated();
    bool has_parking_h_bridge_been_updated = mcpwm_dc_has_parking_h_bridge_been_updated();

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
    commands_printf("    Duty cycle: %.3f", (double)duty_now_parking_brake);
    commands_printf("    Current: %.3f A", (double)curr_pb);
    commands_printf("    Current filtered: %.3f A", (double)curr_pb_filt);
    commands_printf("    State: %d", state_pb);
    commands_printf("    Engaged: %d", is_parking_brake_engaged ? 1 : 0);
    commands_printf("    was_parking_h_bridge_updated: %d", was_parking_h_bridge_updated ? 1 : 0);
    commands_printf("    has_parking_h_bridge_been_updated: %d", has_parking_h_bridge_been_updated ? 1 : 0);
    commands_printf("  Timers:");
    commands_printf("    TIM1 ARR: %u", TIM1->ARR);
    commands_printf("    TIM1 CCR1: %u", TIM1->CCR1);
    commands_printf("    TIM1 CCR2: %u", TIM1->CCR2);
    commands_printf("    TIM1 CCR3: %u", TIM1->CCR3);
    commands_printf("    TIM1 CCR4: %u", TIM1->CCR4);
    commands_printf("    TIM8 CCR1: %u", TIM8->CCR1);
    commands_printf("    TIM8 CCR2: %u", TIM8->CCR2);
    commands_printf("    TIM8 CCR3: %u", TIM8->CCR3);
    commands_printf("    TIM1 CNT: %u", TIM1->CNT);
    commands_printf("  HW config:");
    commands_printf(
        "    [CH1] OC1M=%u | CH1=%s | CH1N=%s",
        (unsigned int)((TIM1->CCMR1 >> 4) & 0x7),
        (TIM1->CCER & TIM_CCER_CC1E) ? "ENABLED" : "DISABLED",
        (TIM1->CCER & TIM_CCER_CC1NE) ? "ENABLED" : "DISABLED");
    commands_printf(
        "    [CH2] OC2M=%u | CH2=%s | CH2N=%s",
        (unsigned int)((TIM1->CCMR1 >> 12) & 0x7),
        (TIM1->CCER & TIM_CCER_CC2E) ? "ENABLED" : "DISABLED",
        (TIM1->CCER & TIM_CCER_CC2NE) ? "ENABLED" : "DISABLED");
    commands_printf(
        "    [CH3] OC3M=%u | CH3=%s | CH3N=%s",
        (unsigned int)((TIM1->CCMR2 >> 4) & 0x7),
        (TIM1->CCER & TIM_CCER_CC3E) ? "ENABLED" : "DISABLED",
        (TIM1->CCER & TIM_CCER_CC3NE) ? "ENABLED" : "DISABLED");
    commands_printf(
        "    [TIM1 MOE] %s (BDTR=0x%04X)",
        (TIM1->BDTR & TIM_BDTR_MOE) ? "ENABLED" : "DISABLED",
        (unsigned int)TIM1->BDTR);
    commands_printf(
        "     [TIM1 CR2] 0x%04X CCPC=%u CCUS=%u",
        (unsigned int)TIM1->CR2,
        (TIM1->CR2 & TIM_CR2_CCPC) ? 1 : 0,
        (TIM1->CR2 & TIM_CR2_CCUS) ? 1 : 0);
    commands_printf(
        "    [TIM1 SR] 0x%04X",
        (unsigned int)TIM1->SR);
    commands_printf(
        "[TIM1 RAW] CR1=%08X CR2=%08X SMCR=%08X DIER=%08X SR=%08X",
        (unsigned int)TIM1->CR1,
        (unsigned int)TIM1->CR2,
        (unsigned int)TIM1->SMCR,
        (unsigned int)TIM1->DIER,
        (unsigned int)TIM1->SR);

    commands_printf(
        "[TIM1 RAW] CCMR1=%08X CCMR2=%08X CCER=%08X BDTR=%08X",
        (unsigned int)TIM1->CCMR1,
        (unsigned int)TIM1->CCMR2,
        (unsigned int)TIM1->CCER,
        (unsigned int)TIM1->BDTR);

    commands_printf(
        "[TIM1 RAW] PSC=%u ARR=%u RCR=%u CCR1=%u CCR2=%u CCR3=%u CCR4=%u CNT=%u",
        (unsigned int)TIM1->PSC,
        (unsigned int)TIM1->ARR,
        (unsigned int)TIM1->RCR,
        (unsigned int)TIM1->CCR1,
        (unsigned int)TIM1->CCR2,
        (unsigned int)TIM1->CCR3,
        (unsigned int)TIM1->CCR4,
        (unsigned int)TIM1->CNT);
}

static void terminal_reset_bools(int argc, const char **argv)
{
    mcpwm_dc_reset_has_parking_h_bridge_been_updated();
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
    terminal_register_command_callback(
        "reset_pb",
        "Reset a parking brake register for debugging",
        NULL,
        terminal_reset_bools);
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
    terminal_unregister_callback(terminal_reset_bools);
#else
#pragma message("DEBUG NOT included")
#endif
}