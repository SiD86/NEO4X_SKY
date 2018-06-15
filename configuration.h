#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

typedef struct {

	uint16_t communication_break_time;
	uint16_t desync_silence_window_time;
	uint16_t send_state_interval;

	uint8_t angle_protect;
	uint16_t ESC_PWM_frequency;

	uint16_t LED_disable_time;
	uint16_t LED_enable_time;

	uint16_t PID_output_limit;
	uint16_t PID_enable_threshold;
	float PID_X[3];
	float I_X_limit;
	float PID_Y[3];
	float I_Y_limit;
	float PID_Z[3];
	float I_Z_limit;

} configuration_t;

// External interface
extern bool configuration_reset();
extern bool configuration_load();
extern void configuration_enter_to_change_mode();

extern configuration_t g_cfg;

#endif /* __CONFIGURATION_H__ */
