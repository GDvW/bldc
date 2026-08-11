#ifndef MCWPM_DC_LOCAL
#define MCWPM_DC_LOCAL

#include "conf_general.h"

extern volatile mc_control_mode control_mode;

extern volatile float dutycycle_set;
extern volatile float current_set;
extern volatile float rpm_set;
extern volatile bool parking_brake_output_set;

extern volatile float dutycycle_now;

extern volatile float switching_frequency_now;
extern volatile float last_current_sample;
extern volatile float last_current_sample_filtered;

extern volatile mc_state state;

extern volatile bool dccal_done;
extern volatile float last_adc_isr_duration;

#endif