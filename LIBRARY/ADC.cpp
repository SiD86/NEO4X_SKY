#include <Arduino.h>
#include "ADC.h"
#define ADC_ENABLE_CHANNEL_COUNT		(10)

static uint16_t g_ADC_buffer[ADC_ENABLE_CHANNEL_COUNT] = { 0 };

void ADC_intialize() {

	// Enable PDC clock
	REG_PMC_PCER1 = PMC_PCER1_PID39;
	while ((REG_PMC_PCSR1 & PMC_PCER1_PID39) == 0);

	pmc_enable_periph_clk(ID_ADC);
	adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
	adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
	adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
	adc_disable_all_channel(ADC);

	// Configure ADC resolution to 12 bits
	REG_ADC_MR &= ~(ADC_MR_LOWRES_BITS_10); 

	// Configure analog inputs
	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	pinMode(A2, INPUT);
	pinMode(A3, INPUT);
	pinMode(A4, INPUT);
	pinMode(A5, INPUT);
	pinMode(A6, INPUT);
	pinMode(A7, INPUT);
	pinMode(A8, INPUT);
	pinMode(A9, INPUT);

	// Enable ADC channels
	REG_ADC_CHER = ADC_CHER_CH0;	// (A7) Reserved
	REG_ADC_CHER = ADC_CHER_CH1;	// (A6) Reserved
	REG_ADC_CHER = ADC_CHER_CH2;	// (A5) Reserved
	REG_ADC_CHER = ADC_CHER_CH3;	// (A4) Reserved
	REG_ADC_CHER = ADC_CHER_CH4;	// (A3) Reserved
	REG_ADC_CHER = ADC_CHER_CH5;	// (A2) Reserved
	REG_ADC_CHER = ADC_CHER_CH6;	// (A1) Reserved
	REG_ADC_CHER = ADC_CHER_CH7;	// (A0) Battery voltage
	REG_ADC_CHER = ADC_CHER_CH10;	// (A8) Reserved
	REG_ADC_CHER = ADC_CHER_CH11;	// (A9) Reserved
}

void ADC_start_conversion() {

	// Initialize PDC channel for ADC
	REG_ADC_PTCR = ADC_PTCR_RXTDIS;
	REG_ADC_RPR  = (uint32_t)g_ADC_buffer;
	REG_ADC_RCR  = ADC_ENABLE_CHANNEL_COUNT;
	REG_ADC_PTCR = ADC_PTCR_RXTEN;

	// Start ADC conversion
	REG_ADC_CR = ADC_CR_START;
}

bool ADC_is_convesrion_complite() {
	return (REG_ADC_RCR == 0);
}

uint16_t ADC_get_data(uint32_t channel_index) {
	return g_ADC_buffer[channel_index] & 0x0FFF;
}
