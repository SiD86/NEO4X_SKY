#include <Arduino.h>
#include "UART.h"
#define TX_PIN								(PIO_PA9)
#define RX_PIN								(PIO_PA8)

static bool g_state = false;

//
// EXTERNAL INTERFACE
//
void UART_initialize(uint32_t speed) {

	// Enable UART clock
	REG_PMC_PCER0 = PMC_PCER0_PID8;
	while ((REG_PMC_PCSR0 & PMC_PCER0_PID8) == 0);

	// Configure TX and RX as A peripheral function
	REG_PIOA_PDR = TX_PIN | RX_PIN;		// Disable PIO control, enable peripheral control

	// Disable PDC channels and reset TX and RX
	REG_UART_PTCR = UART_PTCR_TXTDIS | UART_PTCR_RXTDIS;
	REG_UART_CR = UART_CR_RSTTX | UART_CR_RSTRX | UART_CR_RSTSTA;

	// Configure 8N1 mode
	REG_UART_MR = US_MR_CHRL_8_BIT | US_MR_PAR_NO | US_MR_NBSTOP_1_BIT | US_MR_USART_MODE_NORMAL | US_MR_USCLKS_MCK | US_MR_CHMODE_NORMAL;

	// Configure baudrate
	REG_UART_BRGR = (SystemCoreClock / speed) / 16;

	// Disable all interrupts
	REG_UART_IDR = 0xFFFFFFFF;
}

void UART_set_state(bool is_enable) {

	if (is_enable)
		REG_UART_CR = US_CR_TXEN | US_CR_RXEN;
	else
		REG_UART_CR = US_CR_TXDIS | US_CR_RXDIS;
	g_state = is_enable;
}

void UART_reset() {
	REG_UART_CR = US_CR_RSTTX | US_CR_RSTRX | US_CR_RSTSTA;
	UART_set_state(g_state);
}

bool UART_write(uint8_t* data, uint32_t size, uint32_t timeout) {

	uint32_t begin = millis();

	// Reset status register
	REG_UART_CR = UART_CR_RSTSTA;

	while (size != 0) {

		// Check timeout
		if (millis() - begin > timeout)
			return false;

		// Wait TXRDY and send byte
		if (REG_UART_SR & UART_SR_TXRDY) {

			REG_UART_THR = *data;

			++data;
			--size;
		}
	}

	return true;
}

void UART_dbg(const char* msg) {
	int size = strlen(msg);
	UART_write((uint8_t*)msg, size, 0xFFFFFFFF);
}

bool UART_read(uint8_t* data, uint32_t size, uint32_t timeout) {

	uint32_t begin = millis();

	// Reset status register
	REG_UART_CR = UART_CR_RSTSTA;

	while (size != 0) {

		// Check timeout
		if (millis() - begin > timeout)
			return false;

		// Check errors
		if (REG_UART_SR & (UART_SR_PARE | UART_SR_FRAME | UART_SR_OVRE))
			return false;

		// Wait RXRDY and read byte
		if (REG_UART_SR & UART_SR_RXRDY) {

			*data = REG_UART_RHR;

			++data;
			--size;
		}
	}

	return true;
}
