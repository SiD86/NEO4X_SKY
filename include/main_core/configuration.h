#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

typedef struct {

	uint16_t communication_break_time;
	uint16_t desync_silence_window_time;
	uint16_t send_state_interval;

	uint8_t angle_protect;
	uint16_t ESC_PWM_frequency;

	uint16_t led_disable_time;
	uint16_t led_enable_time;

	uint16_t pid_output_limit;
	uint16_t pid_enable_threshold;
	
	float rate_pid_x[3];
	float rate_i_x_limit;
	float rate_pid_y[3];
	float rate_i_y_limit;
	float rate_pid_z[3];
	float rate_i_z_limit;
	
	float angle_pid_x[3];
	float angle_i_x_limit;
	float angle_pid_y[3];
	float angle_i_y_limit;
	float angle_pid_z[3];
	float angle_i_z_limit;

} configuration_t;

// External interface
extern bool configuration_reset(void);
extern bool configuration_load(void);
extern void configuration_enter_to_change_mode(void);

extern configuration_t g_cfg;

#endif /* __CONFIGURATION_H__ */
