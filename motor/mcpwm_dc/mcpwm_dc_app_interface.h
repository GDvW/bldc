#ifndef MCPWM_DC_APP_INTERFACE_H
#define MCPWM_DC_APP_INTERFACE_H

typedef void (*callback_t)(void);

// Sets the function called when current measurements have been taken
// Setting this to 0 disables the callback
void mcpwm_dc_app_set_measurement_callback(callback_t cb);

bool mcpwm_dc_app_is_speed_control_active(void);

void mcpwm_dc_app_set_current_rpm(float rpm);
void mcpwm_dc_app_set_duty(float dutycycle);
void mcpwm_dc_app_set_current(float current);

#endif