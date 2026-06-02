#pragma once

typedef enum {
    DC_STATE_OFF = 0,
    DC_STATE_STARTING,
    DC_STATE_RUNNING,
    DC_STATE_BRAKING,
    DC_STATE_FULL_BRAKE
} dc_state_t;

typedef enum {
    DC_DIR_REVERSE = -1,
    DC_DIR_FORWARD = 1
} dc_direction_t;

typedef enum {
    DC_MODE_NONE = 0,
    DC_MODE_DUTY,
    DC_MODE_CURRENT,
    DC_MODE_CURRENT_BRAKE,
    DC_MODE_SPEED
} dc_control_mode_t;