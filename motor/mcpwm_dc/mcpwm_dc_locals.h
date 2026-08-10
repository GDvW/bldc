#ifndef MCWPM_DC_LOCAL
#define MCWPM_DC_LOCAL

#include "conf_general.h"

extern volatile mc_control_mode control_mode;

extern volatile float dutycycle_set;
extern volatile float current_set;
extern volatile float rpm_set;

extern volatile float dutycycle_now;

#endif