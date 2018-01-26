#ifndef __PID_H__
#define __PID_H__

static const uint32_t PID_CHANNEL_X = 0;
static const uint32_t PID_CHANNEL_Y = 1;
static const uint32_t PID_CHANNEL_Z = 2;

void PID_initialize(uint32_t ch, uint32_t interval, float output_min, float output_max, float I_min, float I_max);
float PID_process(uint32_t ch, float input, float set_point);
float PID_get_last_output(uint32_t ch);
void PID_set_tunings(uint32_t ch, float Kp, float Ki, float Kd);
void PID_reset(uint32_t ch);

#endif /* __PID_H__ */

