/**
 * Contains definitions for all variables used in the module but not approachable from outside
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

/**
 * Initializes all local variables to a default state
 */
void mcpwm_dc_init_locals()
{
    control_mode = CONTROL_MODE_NONE;
    state = MC_STATE_OFF;

    speed_control_active = false;

    dutycycle_set = 0.0;
    current_set = 0.0;
    rpm_set = 0.0;
    parking_brake_output_set = false;

    dutycycle_now = 0.0;
    direction = DIRECTION_FORWARD;
    rpm_now = 0.0;
    switching_frequency_now = conf->m_dc_f_sw;

    was_h_bridge_configured = false;

    last_current_sample = 0.0;
    last_current_sample_filtered = 0.0;

    last_adc_inj_isr_duration = -1.0;
    last_adc_isr_duration = -1.0;

    init_done = false;
    dccal_done = false;

    filter_create_fir_lowpass((float *)current_fir_coeffs, CURR_FIR_FCUT, CURR_FIR_TAPS_BITS, 1);

    // Create current FIR filter
    filter_create_fir_lowpass((float *)current_fir_coeffs, CURR_FIR_FCUT, CURR_FIR_TAPS_BITS, 1);

    // Create amplitude FIR filter
    filter_create_fir_lowpass((float *)amp_fir_coeffs, AMP_FIR_FCUT, AMP_FIR_TAPS_BITS, 1);
}

// The motor configuration
volatile mc_configuration *conf;
// Control mode of device, e.g. speed, current or none
volatile mc_control_mode control_mode;
// State of device: running, full_brake or none
volatile mc_state state;

// For the speed control app
// Whether speed control is being used or if it should be deactivated
volatile bool speed_control_active;

// Setpoints
volatile float dutycycle_set;
volatile float current_set;
volatile float rpm_set;
volatile bool parking_brake_output_set;

// Real values
volatile float dutycycle_now;
volatile direction_t direction;
volatile float rpm_now;
volatile float switching_frequency_now;

// Current measurements
volatile float last_current_sample;
volatile float last_current_sample_filtered;

// Flags
volatile bool init_done;
volatile bool dccal_done;
volatile float last_adc_isr_duration;
volatile float last_adc_inj_isr_duration;

volatile bool was_h_bridge_configured;

// Filters
// Current FIR filter
volatile float current_fir_coeffs[CURR_FIR_LEN];
volatile float current_fir_samples[CURR_FIR_LEN];
volatile int current_fir_index;

// Amplitude FIR filter
volatile float amp_fir_coeffs[AMP_FIR_LEN];
volatile float amp_fir_samples[AMP_FIR_LEN];
volatile int amp_fir_index;