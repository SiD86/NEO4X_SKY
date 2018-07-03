#ifndef LED_H_
#define LED_H_

extern void led_initialize(void);
extern void led_configuration_mode_enable(void);
extern void led_process(uint32_t main_core_status, uint32_t fly_core_status);

#endif /* LED_H_ */