#ifndef _PID_CONTROLLER_H
#define _PID_CONTROLLER_H

#define PID_CHANNEL_X			(0)
#define PID_CHANNEL_Y			(1)
#define PID_CHANNEL_Z			(2)

extern void pid_initialize(uint32_t ch, float output_limit, float I_limit);
extern float pid_calculate(uint32_t ch, float input, float set_point);
extern float pid_get_last_output(uint32_t ch);
extern void pid_set_tunings(uint32_t ch, float Kp, float Ki, float Kd);
extern void pid_reset_all_channels();

#endif /* _PID_CONTROLLER_H */

