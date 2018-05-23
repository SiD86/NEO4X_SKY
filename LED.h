#ifndef __LED_H__
#define __LED_H__

void LED_initialize();
void LED_configuration_mode_enable();
void LED_process(uint32_t main_core_status, uint32_t fly_core_status);

#endif /* __LED_H__ */