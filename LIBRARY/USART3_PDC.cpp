#include <Arduino.h>
#include "USART3_PDC.h"
#define MAX_BUFFER_SIZE						(64)

static void config_PMC();
static void config_GPIO();

static uint8_t g_tx_buffer[MAX_BUFFER_SIZE] = { 0 };
static uint8_t g_rx_buffer[MAX_BUFFER_SIZE] = { 0 };


//
// EXTERNAL INTERFACE
//
void USART3_initialize(uint32_t speed) {

	config_PMC();
	config_GPIO();

	// Disable PDC channels and reset TX and RX
	REG_USART3_PTCR = US_PTCR_TXTDIS | US_PTCR_RXTDIS;
	REG_USART3_CR = US_CR_RSTTX | US_CR_RSTRX | US_CR_RSTSTA;

	// Configure 8N1 mode
	REG_USART3_MR = US_MR_CHRL_8_BIT | US_MR_PAR_NO | US_MR_NBSTOP_1_BIT | US_MR_USART_MODE_NORMAL | US_MR_USCLKS_MCK | US_MR_CHMODE_NORMAL;

	// Configure baudrate
	REG_USART3_BRGR = (SystemCoreClock / speed) / 16;

	// Disable all interrupts
	REG_USART3_IDR = 0xFFFFFFFF;

	// Configure PDC channels
	REG_USART3_TCR = 0;
	REG_USART3_TPR = (uint32_t)g_tx_buffer;
	REG_USART3_RCR = 0;
	REG_USART3_RPR = (uint32_t)g_rx_buffer;

	// Enable TX and RX
	REG_USART3_CR = US_CR_TXEN | US_CR_RXEN;
}

bool USART3_is_error() {
	uint32_t reg = USART3->US_CSR;
	return (reg & (US_CSR_OVRE | US_CSR_FRAME | US_CSR_PARE));
}

void USART3_reset(bool tx, bool rx) {

	if (tx == true) {
		// Disable PDC channel and reset TX
		REG_USART3_PTCR = US_PTCR_TXTDIS;
		REG_USART3_CR = US_CR_RSTTX | US_CR_RSTSTA;

		// Reset PDC channel
		REG_USART3_TCR = 0;
		REG_USART3_TPR = (uint32_t)g_tx_buffer;

		// Enable TX
		REG_USART3_CR = US_CR_TXEN;
	}

	if (rx == true) {
		// Disable PDC channel and reset RX
		REG_USART3_PTCR = US_PTCR_RXTDIS;
		REG_USART3_CR = US_CR_RSTRX | US_CR_RSTSTA;

		// Reset PDC channel
		REG_USART3_RCR = 0;
		REG_USART3_RPR = (uint32_t)g_rx_buffer;

		// Enable RX
		REG_USART3_CR = US_CR_RXEN;
	}
}


void USART3_TX_start(uint32_t size) {

	if (REG_USART3_TCR != 0)
		return;

	// Initialize DMA for transfer 
	REG_USART3_PTCR = US_PTCR_TXTDIS;
	REG_USART3_TPR = (uint32_t)g_tx_buffer;
	REG_USART3_TCR = size;
	REG_USART3_PTCR = US_PTCR_TXTEN;
}

bool USART3_TX_is_complete() {
	uint32_t reg = REG_USART3_CSR;
	return (reg & US_CSR_TXEMPTY);
}

uint8_t* USART3_TX_get_buffer_address() {
	return g_tx_buffer;
}


void USART3_RX_start(uint32_t size) {

	if (REG_USART3_RCR != 0)
		return;

	// Initialize DMA for receive
	REG_USART3_PTCR = US_PTCR_RXTDIS;
	REG_USART3_RPR = (uint32_t)g_rx_buffer;
	REG_USART3_RCR = size;
	REG_USART3_PTCR = US_PTCR_RXTEN;
}

bool USART3_RX_is_complete() {
	return (REG_USART3_RCR == 0);
}

uint8_t* USART3_RX_get_buffer_address() {
	return g_rx_buffer;
}


//
// INTERNAL INTERFACE
//
static void config_PMC() {

	// Enable clock PIOD 
	REG_PMC_PCER0 |= PMC_PCER0_PID14;
	while ((REG_PMC_PCSR0 & PMC_PCER0_PID14) == 0);

	// Enable clock USART3
	REG_PMC_PCER0 |= PMC_PCER0_PID20;
	while ((REG_PMC_PCSR0 & PMC_PCER0_PID20) == 0);

	// Enable clock PDC
	REG_PMC_PCER1 |= PMC_PCER1_PID39;
	while ((REG_PMC_PCSR1 & PMC_PCER1_PID39) == 0);
}

static void config_GPIO() {

	// Configure RX pin (PD5)
	REG_PIOD_PDR |= PIO_PDR_P5;		// Disable PIO control, enable peripheral control
	REG_PIOD_ABSR |= PIO_ABSR_P5;	// Set peripheral B function

	// Configure TX pin (PD4)
	REG_PIOD_PDR |= PIO_PDR_P4;		// Disable PIO control, enable peripheral control
	REG_PIOD_ABSR |= PIO_ABSR_P4;	// Set peripheral B function
}
