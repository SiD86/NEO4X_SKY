#include <Arduino.h>

uint32_t max_loop_time = 0;

int main() {

	// Disable Watch Dog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;

	init();

	delay(1);

	setup();

	while (true) {

		uint32_t begin = micros();

		loop();

		// Debug
		uint32_t end = micros() - begin;
		if (end > max_loop_time)
			max_loop_time = end;
	}

	return 0;
}
