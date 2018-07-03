#ifndef SYSTIMER_H_
#define SYSTIMER_H_

extern void systimer_initialize(void);
extern uint32_t millis(void);
extern uint32_t micros(void);
extern void delay(uint32_t ms);

#endif /* SYSTIMER_H_ */