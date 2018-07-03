#include <stdbool.h>
#include "sam3x8e.h"
#include "systimer.h"

static volatile uint32_t systime_ms = 0;

void systimer_initialize(void) {
	
	if (SysTick_Config(SystemCoreClock / 1000)) {
		while (true);
	}
}

uint32_t millis(void) {
	return systime_ms;
}

uint32_t micros(void) {
	
     uint32_t ticks = 0;
     uint32_t count = 0;

     SysTick->CTRL;
     do {
         ticks = SysTick->VAL;
         count = systime_ms;
     } 
	 while (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);

     return count * 1000 + (SysTick->LOAD + 1 - ticks) / (SystemCoreClock / 1000000) ;
}

void delay(uint32_t ms) {
	
	uint32_t start = systime_ms;
	while (systime_ms - start < ms);
}




void SysTick_Handler(void) {
	++systime_ms;
}