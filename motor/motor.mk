CSRC += \
	motor/foc_math.c \
	motor/mc_interface.c \
	motor/mcpwm.c \
	motor/mcpwm_foc.c \
	motor/virtual_motor.c \
	motor/mcpwm_dc/mcpwm_dc.c \
	motor/mcpwm_dc/mcpwm_dc_actuation.c \
	motor/mcpwm_dc/mcpwm_dc_control_loop.c \
	motor/mcpwm_dc/mcpwm_dc_getters.c \
	motor/mcpwm_dc/mcpwm_dc_hw.c \
	motor/mcpwm_dc/mcpwm_dc_interrupts.c \
	motor/mcpwm_dc/mcpwm_dc_locals.c \
	motor/mcpwm_dc/mcpwm_dc_measurements.c \
	motor/mcpwm_dc/mcpwm_dc_setters.c \
	motor/mcpwm_dc/mcpwm_dc_app_interface.c \
	motor/mcpwm_dc/mcpwm_dc_control_loop_parking_brake.c \

INCDIR += motor \
	motor/motor_dc