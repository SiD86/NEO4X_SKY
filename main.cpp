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
/*
static void initialize_UART(void) {

	// Enable clock PIOA
	REG_PMC_PCER0 |= PMC_PCER0_PID11;
	while ((REG_PMC_PCSR0 & PMC_PCER0_PID11) == 0);

	// Enable clock UART
	REG_PMC_PCER0 |= PMC_PCER0_PID8;
	while ((REG_PMC_PCSR0 & PMC_PCER0_PID8) == 0);


	// Configure RX pin (PA8)
	REG_PIOA_PDR |= PIO_PDR_P8;			// Disable PIO control, enable peripheral control
	REG_PIOA_ABSR &= ~(PIO_ABSR_P8);	// Set peripheral A function
	REG_PIOA_PUER |= PIO_PUER_P8;		// Enable pullup for RX0

	// Configure TX pin (PA9)
	REG_PIOA_PDR |= PIO_PDR_P9;			// Disable PIO control, enable peripheral control
	REG_PIOA_ABSR |= ~(PIO_ABSR_P9);	// Set peripheral A function
}

static void initialize_ADC() {

	REG_PMC_PCER1 |= PMC_PCER1_PID37; // Enable ADC clock

	adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
	adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
	adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
	adc_disable_all_channel(ADC);
}*/
/*
static void initialize(void) {

	SystemInit();

	// Set Systick to 1ms
	while (SysTick_Config(SystemCoreClock / 1000));

	// Initialize C library
	__libc_init_array();

	// Disable pull-up on every pin
	for (unsigned i = 0; i < PINS_COUNT; i++)
		digitalWrite(i, LOW);

	// Enable parallel access on PIO output data registers
	REG_PIOA_OWER = 0xFFFFFFFF;
	REG_PIOB_OWER = 0xFFFFFFFF;
	REG_PIOC_OWER = 0xFFFFFFFF;
	REG_PIOD_OWER = 0xFFFFFFFF;

	initialize_UART();

	initialize_ADC();
}*/