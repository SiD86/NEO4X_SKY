#include "Arduino.h"
#include "CONFIG.h"		
#include "PID_v1.h"
#define MAX_CHANNEL_COUNT				(3)

struct pid_param_t {

	// Settings
	float Kp;
	float Ki;
	float Kd;
	uint32_t interval;
	float output_min;
	float output_max;
	float integral_min;
	float integral_max;

	// For internal
	uint32_t prev_process_time;
	float integral;
	float prev_input;
	float output;
};

static pid_param_t g_PID_ch[MAX_CHANNEL_COUNT];

void PID_initialize(uint32_t ch, uint32_t interval, float output_min, float output_max, float I_min, float I_max) {
	
	g_PID_ch[ch].Kp = 0;
	g_PID_ch[ch].Ki = 0;
	g_PID_ch[ch].Kd = 0;
	g_PID_ch[ch].interval = interval;
	g_PID_ch[ch].output_min = output_min;
	g_PID_ch[ch].output_max = output_max;
	g_PID_ch[ch].integral_min = I_min;
	g_PID_ch[ch].integral_max = I_max;
	
	PID_reset(ch);
}

float PID_process(uint32_t ch, float set_point, float input) {

	unsigned long current_time = micros();
	uint32_t dT = (current_time - g_PID_ch[ch].prev_process_time);

	if (dT >= g_PID_ch[ch].interval) {

		REG_PIOC_SODR = PIO_SODR_P24;

		// Calculate error
		double error = set_point - input; 

		// Calculate I
		g_PID_ch[ch].integral += (g_PID_ch[ch].Ki * error);
		if (g_PID_ch[ch].integral > g_PID_ch[ch].integral_max) {
			g_PID_ch[ch].integral = g_PID_ch[ch].integral_max;
		}
		else if (g_PID_ch[ch].integral < g_PID_ch[ch].integral_min) {
			g_PID_ch[ch].integral = g_PID_ch[ch].integral_min;
		}

		double dInput = (input - g_PID_ch[ch].prev_input);

		// Calculate PID output
		g_PID_ch[ch].output = g_PID_ch[ch].Kp * error + g_PID_ch[ch].integral - g_PID_ch[ch].Kd * dInput;
		if (g_PID_ch[ch].output > g_PID_ch[ch].output_max) {
			g_PID_ch[ch].output = g_PID_ch[ch].output_max;
		}
		else if (g_PID_ch[ch].output < g_PID_ch[ch].output_min) {
			g_PID_ch[ch].output = g_PID_ch[ch].output_min;
		}

		// Remember variables for next time
		g_PID_ch[ch].prev_input = input;
		g_PID_ch[ch].prev_process_time = current_time;

		REG_PIOC_CODR = PIO_CODR_P24;
	}

	return g_PID_ch[ch].output;
}

void PID_set_tunings(uint32_t ch, float Kp, float Ki, float Kd) {

	g_PID_ch[ch].Kp = Kp;
	g_PID_ch[ch].Ki = Ki;
	g_PID_ch[ch].Kd = Kd;
}

void PID_reset(uint32_t ch) {

	g_PID_ch[ch].prev_process_time = 0;
	g_PID_ch[ch].integral = 0;
	g_PID_ch[ch].prev_input = 0;
	g_PID_ch[ch].output = 0;
}

