#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mc_interface.h"
#include "digital_filter.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "ledpwm.h"
#include "terminal.h"
#include "timeout.h"
#include "encoder/encoder.h"
#include "timer.h"
#include "mcpwm_dc.h"
#include "mcpwm_dc_locals.h"
#include "mcpwm_dc_app_interface.h"

bool mcpwm_dc_app_is_speed_control_active(void)
{
    return speed_control_active;
}

void mcpwm_dc_app_set_current_rpm(float rpm)
{
    rpm_now = rpm;
}

void mcpwm_dc_app_set_duty(float dutycycle)
{
    control_mode = CONTROL_MODE_DUTY;
    mcpwm_dc_set_duty_hl(dutycycle);
}

void mcpwm_dc_app_set_current(float current)
{
    control_mode = CONTROL_MODE_CURRENT;
    mcpwm_dc_set_current_hl(current);
}