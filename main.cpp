#include <Arduino.h>

uint32_t max_loop_time = 0;
extern "C" {
	static void initialize(void);
}

int main() {

	// Disable Watch Dog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;

	//init();
	initialize();

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

#ifdef __cplusplus
extern "C" {
#endif

	void __libc_init_array(void);

	static void initialize(void)
	{
		SystemInit();

		// Set Systick to 1ms interval, common to all SAM3 variants
		if (SysTick_Config(SystemCoreClock / 1000))
		{
			// Capture error
			while (true);
		}

		// Initialize C library
		__libc_init_array();

		// Disable pull-up on every pin
		for (unsigned i = 0; i < PINS_COUNT; i++)
			digitalWrite(i, LOW);

		// Enable parallel access on PIO output data registers
		PIOA->PIO_OWER = 0xFFFFFFFF;
		PIOB->PIO_OWER = 0xFFFFFFFF;
		PIOC->PIO_OWER = 0xFFFFFFFF;
		PIOD->PIO_OWER = 0xFFFFFFFF;

		// Initialize Serial port U(S)ART pins
		PIO_Configure(
			g_APinDescription[PINS_UART].pPort,
			g_APinDescription[PINS_UART].ulPinType,
			g_APinDescription[PINS_UART].ulPin,
			g_APinDescription[PINS_UART].ulPinConfiguration);
		digitalWrite(0, HIGH); // Enable pullup for RX0
		/*PIO_Configure(
			g_APinDescription[PINS_USART0].pPort,
			g_APinDescription[PINS_USART0].ulPinType,
			g_APinDescription[PINS_USART0].ulPin,
			g_APinDescription[PINS_USART0].ulPinConfiguration);
		PIO_Configure(
			g_APinDescription[PINS_USART1].pPort,
			g_APinDescription[PINS_USART1].ulPinType,
			g_APinDescription[PINS_USART1].ulPin,
			g_APinDescription[PINS_USART1].ulPinConfiguration);
		PIO_Configure(
			g_APinDescription[PINS_USART3].pPort,
			g_APinDescription[PINS_USART3].ulPinType,
			g_APinDescription[PINS_USART3].ulPin,
			g_APinDescription[PINS_USART3].ulPinConfiguration);*/

		// Initialize USB pins
		/*PIO_Configure(
			g_APinDescription[PINS_USB].pPort,
			g_APinDescription[PINS_USB].ulPinType,
			g_APinDescription[PINS_USB].ulPin,
			g_APinDescription[PINS_USB].ulPinConfiguration);

		// Initialize CAN pins
		PIO_Configure(
			g_APinDescription[PINS_CAN0].pPort,
			g_APinDescription[PINS_CAN0].ulPinType,
			g_APinDescription[PINS_CAN0].ulPin,
			g_APinDescription[PINS_CAN0].ulPinConfiguration);
		PIO_Configure(
			g_APinDescription[PINS_CAN1].pPort,
			g_APinDescription[PINS_CAN1].ulPinType,
			g_APinDescription[PINS_CAN1].ulPin,
			g_APinDescription[PINS_CAN1].ulPinConfiguration);*/

		// Initialize Analog Controller
		pmc_enable_periph_clk(ID_ADC);
		adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
		adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
		adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
		adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
		adc_disable_all_channel(ADC);

		// Initialize analogOutput module
		analogOutputInit();
	}

#ifdef __cplusplus
}
#endif