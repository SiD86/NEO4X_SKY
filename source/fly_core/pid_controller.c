#include "sam3x8e.h"
#include "pid_controller.h"
#include "systimer.h"
#define MAX_CHANNEL_COUNT				(6)

typedef struct {

	// Settings
	float Kp;
	float Ki;
	float Kd;
	float output_min;
	float output_max;
	float integral_min;
	float integral_max;

	// For internal
	float integral;
	float prev_input;
	uint32_t prev_process_time;
	float output;
} pid_param_t;

static pid_param_t g_PID_ch[MAX_CHANNEL_COUNT];


//
// EXTERNAL INTERFACE
//
void pid_initialize(uint32_t ch, float output_limit, float I_limit) {
	
	g_PID_ch[ch].Kp = 0;
	g_PID_ch[ch].Ki = 0;
	g_PID_ch[ch].Kd = 0;
	g_PID_ch[ch].output_min = -output_limit;
	g_PID_ch[ch].output_max = +output_limit;
	g_PID_ch[ch].integral_min = -I_limit;
	g_PID_ch[ch].integral_max = +I_limit;
	g_PID_ch[ch].integral = 0;
	g_PID_ch[ch].prev_input = 0;
	g_PID_ch[ch].prev_process_time = micros();
	g_PID_ch[ch].output = 0;
}

float pid_calculate(uint32_t ch, float input, float set_point) {

	// Calculate error and time
	uint32_t current_time = micros();
	float error = set_point - input; 
	float dt = (current_time - g_PID_ch[ch].prev_process_time) / 1000000.0; // is seconds
	
	if (dt == 0) // Div 0 check
		dt = g_PID_ch[ch].output;
	
	// Calculate P
	float P = g_PID_ch[ch].Kp * error;

	// Calculate I
	g_PID_ch[ch].integral += (g_PID_ch[ch].Ki * error) * dt;
	if (g_PID_ch[ch].integral > g_PID_ch[ch].integral_max) {
		g_PID_ch[ch].integral = g_PID_ch[ch].integral_max;
	}
	else if (g_PID_ch[ch].integral < g_PID_ch[ch].integral_min) {
		g_PID_ch[ch].integral = g_PID_ch[ch].integral_min;
	}

	// Calculate D
	float D = g_PID_ch[ch].Kd * ((input - g_PID_ch[ch].prev_input) / dt);

	// Calculate PID output
	g_PID_ch[ch].output = P + g_PID_ch[ch].integral - D;
	if (g_PID_ch[ch].output > g_PID_ch[ch].output_max) {
		g_PID_ch[ch].output = g_PID_ch[ch].output_max;
	}
	else if (g_PID_ch[ch].output < g_PID_ch[ch].output_min) {
		g_PID_ch[ch].output = g_PID_ch[ch].output_min;
	}

	// Remember variables
	g_PID_ch[ch].prev_input = input;
	g_PID_ch[ch].prev_process_time = current_time;

	return g_PID_ch[ch].output;
}

float pid_get_last_output(uint32_t ch) {
	return g_PID_ch[ch].output;
}

void pid_set_tunings(uint32_t ch, float Kp, float Ki, float Kd) {
	g_PID_ch[ch].Kp = Kp;
	g_PID_ch[ch].Ki = Ki;
	g_PID_ch[ch].Kd = Kd;
}

void pid_reset_all_channels() {

	for (uint8_t i = 0; i < MAX_CHANNEL_COUNT; ++i) {
		g_PID_ch[i].integral = 0;
		g_PID_ch[i].prev_input = 0;
		g_PID_ch[i].prev_process_time = micros();
		g_PID_ch[i].output = 0;
	}
}
