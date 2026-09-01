#ifndef MCPWM_DC_APP_INTERFACE_H
#define MCPWM_DC_APP_INTERFACE_H

typedef void (*callback_t)(int value);

void mcpwm_dc_app_add_measurement_cb(callback_t cb);

bool mcpwm_dc_app_is_speed_control_active(void);

void mcpwm_dc_app_set_current_rpm(float rpm);
void mcpwm_dc_app_set_duty(float dutycycle);
void mcpwm_dc_app_set_current(float current);

#endif