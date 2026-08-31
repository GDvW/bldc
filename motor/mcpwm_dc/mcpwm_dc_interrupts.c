/**
 * This file contains all interrupts for the motor driver
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

void mcpwm_dc_adc_inj_int_handler(void)
{
    // Start timer for time keeping
    uint32_t t_start = timer_time_now();

    // Do current measuring
    float curr0, curr1, curr2;
    read_currents_raw(&curr0, &curr1, &curr2);
    process_current_measurements(curr0, curr1, curr2);

    last_adc_inj_isr_duration = timer_seconds_elapsed_since(t_start);
}

/*
 * New ADC samples ready in DMA. Do commutation!
 */
void mcpwm_dc_adc_int_handler(void *p, uint32_t flags)
{
    // Suppress unused parameter warnings
    (void)p;
    (void)flags;

    // Time the execution time of the function
    uint32_t t_start = timer_time_now();

    // Update the timer settings (pwm and adc timing) if possible and needed
    update_timer_attempt();

    // Reset the watchdog
    timeout_feed_WDT(THREAD_MCPWM);

    // Update the h bridge to match the currently defined direction
    // This will only be done if the current state is running
    const bool h_bridge_updated = update_h_bridge();
    was_h_bridge_configured = h_bridge_updated;
    take_motor_voltage_measurement(h_bridge_updated);

    // Only create new duty cycles if the motor is running
    // and the FETs are configured correctly.
    // (If they have been configured this iteration, measurements are not correct, so wait till the next time)
    if (state == MC_STATE_RUNNING && !h_bridge_updated)
    {
        run_control_loop();
    }

    float dt = 1.0 / switching_frequency_now;
    mc_interface_mc_timer_isr(false, dt);

    last_adc_isr_duration = timer_seconds_elapsed_since(t_start);
}