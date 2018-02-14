#include "Arduino.h"
#include "CONFIG.h"		
#include "PID_v1.h"
#include "util.h"
#define MAX_CHANNEL_COUNT				(3)

struct pid_param_t {

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
	float prev_error;
	uint32_t prev_process_time;
	float output;
};

static pid_param_t g_PID_ch[MAX_CHANNEL_COUNT];

uint32_t g_PID_OOR_count = 0;

void PID_initialize(uint32_t ch, float output_limit, float I_limit) {
	
	g_PID_ch[ch].Kp = 0;
	g_PID_ch[ch].Ki = 0;
	g_PID_ch[ch].Kd = 0;
	g_PID_ch[ch].output_min = -output_limit;
	g_PID_ch[ch].output_max = +output_limit;
	g_PID_ch[ch].integral_min = -I_limit;
	g_PID_ch[ch].integral_max = +I_limit;
	
	PID_reset(ch);
}

float PID_process(uint32_t ch, float input, float set_point) {

	// Calculate error and time
	float error = set_point - input; 
	uint32_t current_time = micros();

	// Calculate P
	float P = g_PID_ch[ch].Kp * error;

	// Calculate I
	g_PID_ch[ch].integral += (g_PID_ch[ch].Ki * error);
	if (g_PID_ch[ch].integral > g_PID_ch[ch].integral_max) {
		g_PID_ch[ch].integral = g_PID_ch[ch].integral_max;
	}
	else if (g_PID_ch[ch].integral < g_PID_ch[ch].integral_min) {
		g_PID_ch[ch].integral = g_PID_ch[ch].integral_min;
	}

	// Calculate D
	float dt = (current_time - g_PID_ch[ch].prev_process_time) / 1000000.0; // is seconds
	float D = g_PID_ch[ch].Kd * ((error - g_PID_ch[ch].prev_error) / dt);

	// Calculate PID output
	g_PID_ch[ch].output = P + g_PID_ch[ch].integral + D;
	if (g_PID_ch[ch].output > g_PID_ch[ch].output_max) {
		g_PID_ch[ch].output = g_PID_ch[ch].output_max;
		++g_PID_OOR_count; // DEBUG
	}
	else if (g_PID_ch[ch].output < g_PID_ch[ch].output_min) {
		g_PID_ch[ch].output = g_PID_ch[ch].output_min;
		++g_PID_OOR_count; // DEBUG
	}

	// Remember variables
	g_PID_ch[ch].prev_error = error;
	g_PID_ch[ch].prev_process_time = current_time;

	return g_PID_ch[ch].output;
}

float PID_get_last_output(uint32_t ch) {
	return g_PID_ch[ch].output;
}

void PID_set_tunings(uint32_t ch, float Kp, float Ki, float Kd) {
	g_PID_ch[ch].Kp = Kp;
	g_PID_ch[ch].Ki = Ki;
	g_PID_ch[ch].Kd = Kd;
}

void PID_reset(uint32_t ch) {
	g_PID_ch[ch].integral = 0;
	g_PID_ch[ch].prev_error = 0;
	g_PID_ch[ch].prev_process_time = micros();
	g_PID_ch[ch].output = 0;
}

