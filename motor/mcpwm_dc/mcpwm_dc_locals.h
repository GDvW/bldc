#ifndef MCWPM_DC_LOCAL
#define MCWPM_DC_LOCAL

#include "conf_general.h"
#include "mcpwm_dc_hw.h"

#define PARKING_BRAKE_ENABLED 1
#define PARKING_BRAKE_CURRENT 500 // mA
#define PARKING_BRAKE_DUTY 0.75
#define PARKING_BRAKE_CONTROL_MODE CONTROL_MODE_DUTY

// Everything with locals
void mcpwm_dc_init_locals(void);

// Measurements
void mcpwm_dc_init_measurements(void);
void do_dc_cal(void);
void update_adc_sample_pos(mc_timer_struct *t);
void read_currents_raw(float *curr0, float *curr1, float *curr2);
void process_current_measurements(float curr0, float curr1, float curr2);
void take_motor_voltage_measurement(bool h_bridge_updated);

// Actuation (bridge between public API and control loop)
void mcpwm_dc_set_duty_hl(float dutycycle);
void mcpwm_dc_set_duty_ll(float dutycycle);
void mcpwm_dc_set_current_hl(float current);
void stop_pwm_ll(void);
void stop_pwm_motor_ll(void);
void full_brake_ll(void);

// Control loops
void run_control_loop(void);
void run_parking_brake_control_loop(void);

typedef enum
{
    DIRECTION_BACKWARD = 0,
    DIRECTION_FORWARD = 1
} direction_t;

// The motor configuration
extern volatile mc_configuration *conf;
// Control mode of device, e.g. speed, current or none
extern volatile mc_control_mode control_mode;
// State of device: running, full_brake or none
extern volatile mc_state state;

// Parking brake state registers
extern volatile mc_state state_parking_brake;
extern volatile float duty_now_parking_brake;

// For the speed control app
// Whether speed control is being used or if it should be deactivated
extern volatile bool speed_control_active;

// Setpoints
extern volatile float dutycycle_set;
extern volatile float current_set;
extern volatile float rpm_set;
extern volatile bool parking_brake_output_set;

// Real values
extern volatile float dutycycle_now;
extern volatile direction_t direction;
extern volatile float rpm_now;
extern volatile float switching_frequency_now;

// Current measurements
extern volatile float last_current_sample;
extern volatile float last_current_sample_filtered;
extern volatile float pb_last_current_sample;
extern volatile float pb_last_current_sample_filtered;

// Flags
extern volatile bool init_done;
extern volatile bool dccal_done;
extern volatile float last_adc_isr_duration;
extern volatile float last_adc_inj_isr_duration;

// DEBUG
extern volatile bool was_h_bridge_configured;

// Callbacks
// Stores the measurement done callback. Meant for the app_interface
extern void (*volatile after_measurement_taken)(void);

// Filters
// Current FIR filter
#define CURR_FIR_TAPS_BITS 4
#define CURR_FIR_LEN (1 << CURR_FIR_TAPS_BITS)
#define CURR_FIR_FCUT 0.15
extern volatile float current_fir_coeffs[CURR_FIR_LEN];
extern volatile float current_fir_samples[CURR_FIR_LEN];
extern volatile int current_fir_index;

// Parking brake FIR filter
#define PB_CURR_FIR_TAPS_BITS 4
#define PB_CURR_FIR_LEN (1 << PB_CURR_FIR_TAPS_BITS)
#define PB_CURR_FIR_FCUT 0.15
extern volatile float pb_current_fir_coeffs[PB_CURR_FIR_LEN];
extern volatile float pb_current_fir_samples[PB_CURR_FIR_LEN];
extern volatile int pb_current_fir_index;

// Amplitude FIR filter
#define AMP_FIR_TAPS_BITS 7
#define AMP_FIR_LEN (1 << AMP_FIR_TAPS_BITS)
#define AMP_FIR_FCUT 0.02
extern volatile float amp_fir_coeffs[AMP_FIR_LEN];
extern volatile float amp_fir_samples[AMP_FIR_LEN];
extern volatile int amp_fir_index;

#endif