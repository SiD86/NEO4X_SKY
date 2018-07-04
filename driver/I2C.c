#include <stdbool.h>
#include <string.h>
#include "sam3x8e.h"
#include "I2C.h"
#include "systimer.h"
#define IS_BIT_SET(_status, _bit)		(((_status) & (_bit)) == (_bit))
#define SDA_PIN							(PIO_PB12)
#define SCK_PIN							(PIO_PB13)
#define MAX_PDC_BUFFER_SIZE				(64)
#define I2C_DRIVER_DEFAULT_TIMEOUT_US	(5000)

static void stop_communication(void);

static uint8_t tx_buffer[MAX_PDC_BUFFER_SIZE] = { 0 };
static uint8_t rx_buffer[MAX_PDC_BUFFER_SIZE] = { 0 };

static uint32_t g_internal_address_length = 1;
static volatile uint32_t g_status = I2C_DRIVER_NO_ERROR;


//
// EXTERNAL INTERFACE
//
void I2C_initialize(uint32_t clock_speed) {

	// Enable PDC clock
	REG_PMC_PCER1 = PMC_PCER1_PID39;
	while ((REG_PMC_PCSR1 & PMC_PCER1_PID39) == 0);

	// Enable I2C and watch timeout timer clock
	REG_PMC_PCER0 = PMC_PCER0_PID23 | PMC_PCER0_PID30;
	while (	(REG_PMC_PCSR0 & (PMC_PCER0_PID23 | PMC_PCER0_PID30)) == 0 );

	// Configure watch timeout timer
	REG_TC1_CMR0 = TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_CPCDIS;
	REG_TC1_RC0 = I2C_DRIVER_DEFAULT_TIMEOUT_US * (SystemCoreClock / 8 / 1000 / 1000);
	NVIC_EnableIRQ(TC3_IRQn);

	// Disable and reset I2C controller
	REG_TWI1_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS; // Disable PDC
	REG_TWI1_CR = TWI_CR_SWRST | TWI_CR_SVDIS | TWI_CR_MSDIS;
	
	// Send 9 pulses on SCL
	REG_PIOB_PER = SCK_PIN;
	REG_PIOB_OER = SCK_PIN;
	for (uint32_t i = 0; i < 9; ++i) {
		
		REG_PIOB_CODR = SCK_PIN;
		delay(1);
		REG_PIOB_SODR = SCK_PIN;
		delay(1);
	}

	// Configure SCK and SDA as A peripheral function
	REG_PIOB_PDR = SDA_PIN | SCK_PIN;					// Disable PIO

	// Configure I2C controller
	REG_TWI1_RHR; // Reset holding register
	REG_TWI1_CWGR = clock_speed;
	REG_TWI1_CR = TWI_CR_MSEN | TWI_CR_SVDIS; 
	
	REG_TWI1_IDR = 0xFFFFFFFF;
	NVIC_EnableIRQ(TWI1_IRQn);
}

void I2C_set_internal_address_length(uint32_t length) {
	g_internal_address_length = length;
}

uint32_t I2C_get_status() {
	return g_status;
}

void I2C_force_reset_error_status() {

	if (g_status == I2C_DRIVER_ERROR)
		g_status = I2C_DRIVER_NO_ERROR;
}

//
// TX
//
bool I2C_write_byte(uint8_t dev_addr, uint32_t internal_addr, uint8_t data) {
	return I2C_write_bytes(dev_addr, internal_addr, &data, 1);
}

bool I2C_write_bytes(uint8_t dev_addr, uint32_t internal_addr, uint8_t* data, uint32_t size) {

	// Disable all current I2C communications
	stop_communication();
	g_status = I2C_DRIVER_NO_ERROR;

	// Copy data to transmit buffer
	memcpy(tx_buffer, data, size);

	// Start send data and wait complete communication
	I2C_async_write_bytes(dev_addr, internal_addr, size);
	while (g_status == I2C_DRIVER_BUSY);

	return (g_status == I2C_DRIVER_NO_ERROR);
}

bool I2C_async_write_bytes(uint8_t dev_addr, uint32_t internal_addr, uint32_t size) {

	// Check driver state and timeout operation
	if (g_status == I2C_DRIVER_BUSY)
		return false;

	// Start watch timeout timer
	REG_TC1_CCR0 = TC_CCR_SWTRG | TC_CCR_CLKEN;
	REG_TC1_IER0 = TC_IER_CPCS;

	// Disable all I2C interrupts
	REG_TWI1_IDR = 0xFFFFFFFF;

	// Configure TX PDC channel
	REG_TWI1_TPR = (uint32_t)tx_buffer;
	REG_TWI1_TCR = size;

	// Configure Master mode (DADR | write mode | internal address length)
	REG_TWI1_MMR = (dev_addr << 16) | (0 << 12) | (g_internal_address_length << 8);
	REG_TWI1_IADR = internal_addr;

	// Enable transmiter
	REG_TWI1_PTCR = TWI_PTCR_TXTEN | TWI_PTCR_RXTDIS;

	// Configure interrupts
	REG_TWI1_IER = TWI_IER_ENDTX | TWI_IER_NACK;

	g_status = I2C_DRIVER_BUSY;
	return true;
}

uint8_t* I2C_async_get_tx_buffer_address(void) {
	return tx_buffer;
}

//
// RX
//
bool I2C_read_byte(uint8_t dev_addr, uint32_t internal_addr, uint8_t* data) {
	return I2C_read_bytes(dev_addr, internal_addr, data, 1);
}

bool I2C_read_bytes(uint8_t dev_addr, uint32_t internal_addr, uint8_t* data, uint32_t size) {

	// Disable all current I2C communications
	stop_communication();
	g_status = I2C_DRIVER_NO_ERROR;

	I2C_async_read_bytes(dev_addr, internal_addr, size);
	while (g_status == I2C_DRIVER_BUSY);

	// Check driver status
	if (g_status == I2C_DRIVER_ERROR)
		return false;

	// Copy recv data
	memcpy(data, rx_buffer, size);
	return true;
}

bool I2C_async_read_bytes(uint8_t dev_addr, uint32_t internal_addr, uint32_t size) {

	// Check driver state
	if (g_status == I2C_DRIVER_BUSY)
		return false;

	// Start watch timeout timer
	REG_TC1_CCR0 = TC_CCR_SWTRG | TC_CCR_CLKEN;
	REG_TC1_IER0 = TC_IER_CPCS;

	// Disable all I2C interrupts
	REG_TWI1_IDR = 0xFFFFFFFF;

	// Configure RX PDC channel
	REG_TWI1_RPR = (uint32_t)rx_buffer;
	REG_TWI1_RCR = size - 1; // Without last byte (for send STOP)

	// Configure Master mode (DADR | read mode | internal addres length)
	REG_TWI1_MMR = (dev_addr << 16) | TWI_MMR_MREAD | (g_internal_address_length << 8);
	REG_TWI1_IADR = internal_addr;

	// Enable reciever
	REG_TWI1_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTEN;

	// Send START
	REG_TWI1_CR = TWI_CR_START;

	// Configure interrupts
	REG_TWI1_IER = TWI_IER_ENDRX | TWI_IER_NACK;

	g_status = I2C_DRIVER_BUSY;
	return true;
}

uint8_t* I2C_async_get_rx_buffer_address(void) {
	return rx_buffer;
}


uint32_t I2C_write_bits(uint8_t dev_addr, uint8_t internal_addr, uint8_t mask, uint8_t bits) {

	uint8_t reg_data = 0;
	if (I2C_read_byte(dev_addr, internal_addr, &reg_data) == false)
		return false;

	reg_data &= ~mask;	// Clear bits
	reg_data |= bits;	// Write new bits

	return I2C_write_byte(dev_addr, internal_addr, reg_data);
}

//
// INTERNAL INTERFACE
//
static void stop_communication(void) {

	// Disable PDC
	REG_TWI1_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;
	REG_TWI1_RCR = 0;
	REG_TWI1_TCR = 0;

	// Disable all I2C interrupts
	REG_TWI1_IDR = 0xFFFFFFFF;

	// Reset status register
	REG_TWI1_SR;

	// Disable watch timeout timer
	REG_TC1_IDR0 = 0xFFFFFFFF;
	REG_TC1_CCR0 = TC_CCR_CLKDIS;
}

//
// IRQ handlers
//
void TWI1_Handler(void) {

	uint32_t status = REG_TWI1_SR;
	uint32_t irq_mask = REG_TWI1_IMR;

	// Errors
	if (IS_BIT_SET(status, TWI_SR_NACK) && IS_BIT_SET(irq_mask, TWI_IMR_NACK)) {
		stop_communication();
		g_status = I2C_DRIVER_ERROR;
	}
	
	// Send STOP complete
	else if (IS_BIT_SET(status, TWI_SR_TXCOMP) && IS_BIT_SET(irq_mask, TWI_IMR_TXCOMP)) {
		stop_communication();
		g_status = I2C_DRIVER_NO_ERROR;
	}

	// TX handler
	else if (IS_BIT_SET(status, TWI_SR_ENDTX) && IS_BIT_SET(irq_mask, TWI_IMR_ENDTX)) {

		// Disable TX PDC channel
		REG_TWI1_PTCR = TWI_PTCR_TXTDIS;

		// Send STOP
		REG_TWI1_CR = TWI_CR_STOP;

		// Configure interrupts
		REG_TWI1_IDR = 0xFFFFFFFF;
		REG_TWI1_IER = TWI_IER_TXCOMP;
	}

	// RX handler
	else if (IS_BIT_SET(status, TWI_SR_ENDRX) && IS_BIT_SET(irq_mask, TWI_IMR_ENDRX)) {

		// Disable RX PDC channel
		REG_TWI1_PTCR = TWI_PTCR_RXTDIS;

		// Send STOP
		REG_TWI1_CR = TWI_CR_STOP;

		// Configure interrupts
		REG_TWI1_IDR = 0xFFFFFFFF;
		REG_TWI1_IER = TWI_IER_RXRDY;
	}

	// Recv last byte
	else if (IS_BIT_SET(status, TWI_SR_RXRDY) && IS_BIT_SET(irq_mask, TWI_IMR_RXRDY)) {

		// Read last byte
		uint8_t* buffer_addr = (uint8_t*)REG_TWI1_RPR;
		*buffer_addr = REG_TWI1_RHR;

		// Configure interrupts
		REG_TWI1_IDR = 0xFFFFFFFF;
		REG_TWI1_IER = TWI_IER_TXCOMP;
	}
}

void TC3_Handler(void) {

	uint32_t status = REG_TC1_SR0;
	uint32_t irq_mask = REG_TC1_IMR0;

	if (IS_BIT_SET(irq_mask, TC_IMR_CPCS) && IS_BIT_SET(status, TC_SR_CPCS)) {
		stop_communication();
		g_status = I2C_DRIVER_ERROR;
	}
}