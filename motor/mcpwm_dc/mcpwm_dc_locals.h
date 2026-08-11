#ifndef MCWPM_DC_LOCAL
#define MCWPM_DC_LOCAL

#include "conf_general.h"

#define PARKING_BRAKE_ENABLED 1
#define PARKING_BRAKE_CURRENT 500 // mA

typedef enum {
    DIRECTION_BACKWARD = 0,
    DIRECTION_FORWARD = 1
} direction_t;

// Control mode of device, e.g. speed, current or none
extern volatile mc_control_mode control_mode;

extern volatile float dutycycle_set;
extern volatile float current_set;
extern volatile float rpm_set;
extern volatile bool parking_brake_output_set;

extern volatile float dutycycle_now;
extern volatile direction_t direction;
extern volatile float switching_frequency_now;

extern volatile float last_current_sample;
extern volatile float last_current_sample_filtered;

// State of device: running, full_brake or none
extern volatile mc_state state;

extern volatile bool init_done; 
extern volatile bool dccal_done;
extern volatile float last_adc_isr_duration;

extern volatile mc_configuration *conf;

void mcpwm_dc_init_locals(void);

void mcpwm_dc_set_duty_cleaned(float duty_cycle);

// Filters
// Current FIR filter
#define CURR_FIR_TAPS_BITS 4
#define CURR_FIR_LEN (1 << CURR_FIR_TAPS_BITS)
#define CURR_FIR_FCUT 0.15
extern volatile float current_fir_coeffs[CURR_FIR_LEN];
extern volatile float current_fir_samples[CURR_FIR_LEN];
extern volatile int current_fir_index;

// Amplitude FIR filter
#define AMP_FIR_TAPS_BITS 7
#define AMP_FIR_LEN (1 << AMP_FIR_TAPS_BITS)
#define AMP_FIR_FCUT 0.02
extern volatile float amp_fir_coeffs[AMP_FIR_LEN];
extern volatile float amp_fir_samples[AMP_FIR_LEN];
extern volatile int amp_fir_index;


#endif