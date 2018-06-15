#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#define PID_CHANNEL_X		(0)
#define PID_CHANNEL_Y		(1)
#define PID_CHANNEL_Z		(2)

void pid_initialize(uint32_t ch, float output_limit, float I_limit);
float pid_calculate(uint32_t ch, float input, float set_point);
float pid_get_last_output(uint32_t ch);
void pid_set_tunings(uint32_t ch, float Kp, float Ki, float Kd);
void pid_reset_all_channels();

#endif /* __PID_CONTROLLER_H__ */

