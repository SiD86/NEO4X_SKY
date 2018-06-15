#ifndef __LED_H__
#define __LED_H__

void led_initialize();
void led_configuration_mode_enable();
void led_process(uint32_t main_core_status, uint32_t fly_core_status);

#endif /* __LED_H__ */