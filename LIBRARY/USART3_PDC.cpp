#include <Arduino.h>
#include "USART3_PDC.h"
#define USART_BAUDRATE						(175000)
#define TX_PIN								(PIO_PD4)
#define RX_PIN								(PIO_PD5)
#define MAX_BUFFER_SIZE						(64)

static uint8_t g_tx_buffer[MAX_BUFFER_SIZE] = { 0 };
static uint8_t g_rx_buffer[MAX_BUFFER_SIZE] = { 0 };


//
// EXTERNAL INTERFACE
//
void USART3_initialize() {

	// Enable USART3 clock 
	REG_PMC_PCER0 = PMC_PCER0_PID20;
	while ((REG_PMC_PCSR0 & PMC_PCER0_PID20) == 0);

	// Enable PDC clock
	REG_PMC_PCER1 = PMC_PCER1_PID39;
	while ( (REG_PMC_PCSR1 & PMC_PCER1_PID39) == 0 );
	
	// Configure TX as output (B peripheral function) without pull-up
	REG_PIOD_PER   = TX_PIN;
	REG_PIOD_OER   = TX_PIN;
	REG_PIOD_PUDR  = TX_PIN;
	REG_PIOD_PDR   = TX_PIN;
	REG_PIOD_ABSR |= TX_PIN;

	// Configure RX as input (B peripheral function) with pull-up
	REG_PIOD_PER   = RX_PIN;
	REG_PIOD_ODR   = RX_PIN;
	REG_PIOD_PUER  = RX_PIN;
	REG_PIOD_PDR   = RX_PIN;
	REG_PIOD_ABSR |= RX_PIN;

	// Disable PDC channels and reset TX and RX
	REG_USART3_PTCR = US_PTCR_TXTDIS | US_PTCR_RXTDIS;
	REG_USART3_CR = US_CR_RSTTX | US_CR_RSTRX | US_CR_RSTSTA;

	// Configure 8N1 mode
	REG_USART3_MR = US_MR_CHRL_8_BIT | US_MR_PAR_NO | US_MR_NBSTOP_1_BIT | US_MR_USART_MODE_NORMAL | US_MR_USCLKS_MCK | US_MR_CHMODE_NORMAL;

	// Configure baudrate
	REG_USART3_BRGR = (SystemCoreClock / USART_BAUDRATE) >> 4;

	// Disable all interrupts
	REG_USART3_IDR = 0xFFFFFFFF;
	NVIC_EnableIRQ(USART1_IRQn);

	// Configure PDC channels
	REG_USART3_TCR = 0;
	REG_USART3_TPR = (uint32_t)g_tx_buffer;
	REG_USART3_RCR = 0;
	REG_USART3_RPR = (uint32_t)g_rx_buffer;

	// Enable TX and RX
	REG_USART3_CR = US_CR_TXEN | US_CR_RXEN;
}

bool USART3_is_error() {
	uint32_t reg = REG_USART3_CSR;
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

		// Disable all interrupts
		REG_USART3_IDR = 0xFFFFFFFF;

		// Reset PDC channel
		REG_USART3_RCR = 0;
		REG_USART3_RPR = (uint32_t)g_rx_buffer;

		// Enable RX
		REG_USART3_CR = US_CR_RXEN;
	}
}


void USART3_start_tx(uint32_t size) {

	// Initialize DMA for transfer 
	REG_USART3_PTCR = US_PTCR_TXTDIS;
	REG_USART3_TPR = (uint32_t)g_tx_buffer;
	REG_USART3_TCR = size;
	REG_USART3_PTCR = US_PTCR_TXTEN;
}

bool USART3_is_tx_complete() {
	uint32_t reg = REG_USART3_CSR;
	return (reg & US_CSR_TXEMPTY);
}

uint8_t* USART3_get_tx_buffer_address() {
	return g_tx_buffer;
}


void USART3_start_rx() {

	// Disable DMA
	REG_USART3_PTCR = US_PTCR_RXTDIS;

	// Initialize frame timeout
	REG_USART3_RTOR = 35;
	REG_USART3_CR |= US_CR_STTTO;
	REG_USART3_IER = US_IER_TIMEOUT;

	// Initialize DMA for receive
	REG_USART3_RPR = (uint32_t)g_rx_buffer;
	REG_USART3_RCR = MAX_BUFFER_SIZE;

	// Enable DMA
	REG_USART3_PTCR = US_PTCR_RXTEN;
}

bool USART3_is_frame_received() {
	return REG_USART3_CSR & US_CSR_TIMEOUT;
}

uint32_t USART3_get_frame_size() {
	return MAX_BUFFER_SIZE - REG_USART3_RCR;
}

uint8_t* USART3_get_rx_buffer_address() {
	return g_rx_buffer;
}


//
// This for only frame timeout IRQ
//
void USART3_Handler() {
	// Disable DMA and frame timeout IRQ
	REG_USART3_IDR = US_IER_TIMEOUT;
	REG_USART3_PTCR = US_PTCR_RXTDIS;
}