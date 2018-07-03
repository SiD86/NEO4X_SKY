#include <stdbool.h>
#include "sam3x8e.h"
#include "ADC.h"
#define ADC_ENABLE_CHANNEL_COUNT		(9)

static uint16_t g_ADC_buffer[ADC_ENABLE_CHANNEL_COUNT] = { 0 };

void ADC_intialize(void) {

	// Enable ADC and PDC clock
	REG_PMC_PCER1 = PMC_PCER1_PID37 | PMC_PCER1_PID39;
	while ( (REG_PMC_PCSR1 & (PMC_PCER1_PID37 | PMC_PCER1_PID39)) == 0 );
	
	// Configure analog inputs
	REG_PIOA_PER = PIO_PER_P16 | PIO_PER_P24 | PIO_PER_P23 | PIO_PER_P22 | PIO_PER_P6 |
	PIO_PER_P4  | PIO_PER_P3  | PIO_PER_P2;
	REG_PIOB_PER = PIO_PER_P17;
	
	REG_PIOA_ODR = PIO_ODR_P16 | PIO_ODR_P24 | PIO_ODR_P23 | PIO_ODR_P22 | PIO_ODR_P6 |
	PIO_ODR_P4  | PIO_ODR_P3  | PIO_ODR_P2;
	REG_PIOB_ODR = PIO_PER_P17;

	// Reset ADC
	REG_ADC_CR = ADC_CR_SWRST;
	REG_ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;

	// Setup ADC
	REG_ADC_MR = ADC_MR_PRESCAL(SystemCoreClock / (2 * 20000000) - 1) | ADC_MR_STARTUP_SUT768 | ADC_MR_SETTLING_AST17 | ADC_MR_TRGEN_DIS;
	
	// Disable all IRQ
	REG_ADC_IDR = 0xFFFFFFFF;
	
	// Disable all channels
	REG_ADC_IDR = 0xFFFFFFFF;

	// Enable ADC channels
	REG_ADC_CHER = ADC_CHER_CH7;	// (A0) Battery voltage (12V)
	//REG_ADC_CHER = ADC_CHER_CH6;	// (A1) GND
	REG_ADC_CHER = ADC_CHER_CH5;	// (A2) Wireless power supply (5V)
	//REG_ADC_CHER = ADC_CHER_CH4;	// (A3) Not use
	REG_ADC_CHER = ADC_CHER_CH3;	// (A4) Camera power supply (5V) 
	REG_ADC_CHER = ADC_CHER_CH2;	// (A5) Sensors power supply (3V3) 
	REG_ADC_CHER = ADC_CHER_CH1;	// (A6) Temperature
	REG_ADC_CHER = ADC_CHER_CH0;	// (A7) Temperature
	REG_ADC_CHER = ADC_CHER_CH10;	// (A8) Temperature
	REG_ADC_CHER = ADC_CHER_CH11;	// (A9) Temperature
	REG_ADC_CHER = ADC_CHER_CH12;	// (A10) Vibration
}

void ADC_start_conversion(void) {

	// Initialize PDC channel for ADC
	REG_ADC_PTCR = ADC_PTCR_RXTDIS;
	REG_ADC_RPR  = (uint32_t)g_ADC_buffer;
	REG_ADC_RCR  = ADC_ENABLE_CHANNEL_COUNT;
	REG_ADC_PTCR = ADC_PTCR_RXTEN;

	// Start ADC conversion
	REG_ADC_CR = ADC_CR_START;
}

bool ADC_is_convesrion_complite(void) {
	return (REG_ADC_RCR == 0);
}

uint16_t ADC_get_data(uint32_t channel_index) {
	return g_ADC_buffer[channel_index] & 0x0FFF;
}
