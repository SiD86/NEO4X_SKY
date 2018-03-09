#include <Arduino.h>
#include "LIBRARY\USART3_PDC.h"
#include "TXRX_PROTOCOL.h"
#include "communication_subsystem.h"
#include "configuration_subsystem.h"
#include "util.h"
#define USART_BAUDRATE						(175000)

static const uint32_t g_packet_size = sizeof(TXRX::fly_controller_packet_t);
static const uint32_t g_data_size = sizeof(TXRX::fly_controller_packet_t::data);

static void process_rx();
static void process_tx();
static bool process_rx_data();
static uint32_t calculate_CRC(const uint8_t* data);

static uint32_t g_status = CSS::NO_ERROR;

TXRX::control_data_t g_cp = {0};
TXRX::state_data_t g_sp = {0};

uint32_t g_hardware_error_count = 0; // DEBUG
uint32_t g_software_error_count = 0; // DEBUG
uint32_t g_desync_count = 0; // DEBUG


//
// EXTERNAL INTERFACE
//
void CSS::initialize() {
	USART3_initialize(USART_BAUDRATE);
	USART3_RX_start(g_packet_size);
}

void CSS::process() {

	if (USART3_is_error() == true) {
		USART3_reset(true, true);
		++g_hardware_error_count;
	}

	process_tx();
	process_rx();
}

uint32_t CSS::get_status() {
	return g_status;
}


//
// INTERNAL INTERFACE
//
static void process_tx() {

	static uint32_t prev_tx_time = 0;

	// Check TX interval
	if (millis() - prev_tx_time < g_cfg.send_state_interval)
		return;

	// Check complite TX previus data
	if (USART3_TX_is_complete() == false)
		return;

	// Build new packet
	TXRX::fly_controller_packet_t* packet = (TXRX::fly_controller_packet_t*)USART3_TX_get_buffer_address();
	memcpy(packet->data, &g_sp, g_data_size);
	packet->CRC = calculate_CRC(packet->data);

	// Start transmit
	USART3_TX_start(g_packet_size);

	// Update time 
	prev_tx_time = millis();
}

static void process_rx() {

	static uint32_t prev_rx_data_time = 0;

	uint32_t current_time = millis();

	if (USART3_RX_is_complete() == true) {

		// Update time
		prev_rx_data_time = current_time;

		// Process data
		if (IS_BIT_CLEAR(g_status, CSS::DESYNC)) {

			static uint32_t error_count = 0;
			if (process_rx_data() == true) {
				error_count = 0;
				CLEAR_STATUS_BIT(g_status, CSS::CONNECTION_LOST);
			}
			else {

				if (++error_count >= 5) {
					SET_STATUS_BIT(g_status, CSS::DESYNC);
					++g_desync_count; // DEBUG
				}
				++g_software_error_count; // DEBUG
			}
		}

		// Initialize start receive next data
		USART3_RX_start(g_packet_size);
	}
	else {

		// Check desync communication
		if (IS_BIT_SET(g_status, CSS::DESYNC)) {

			// Wait silence window and reset receiver
			if (current_time - prev_rx_data_time > g_cfg.desync_silence_window_time) {
				USART3_reset(false, true);
				USART3_RX_start(g_packet_size);
				CLEAR_STATUS_BIT(g_status, CSS::DESYNC);
			}
		}

		// Check communication lost
		if (current_time - prev_rx_data_time > g_cfg.connection_lost_timeout) {
			SET_STATUS_BIT(g_status, CSS::CONNECTION_LOST);
		}
	}
}

static bool process_rx_data() {

	TXRX::fly_controller_packet_t* packet = (TXRX::fly_controller_packet_t*)USART3_RX_get_buffer_address();

	// Check CRC
	uint32_t CRC = calculate_CRC(packet->data);
	if (packet->CRC != CRC)
		return false;

	// Copy data
	memcpy(&g_cp, packet->data, g_data_size); 
	return true;
}

/*static uint8_t calculate_CRC8(const uint8_t* data) {

	uint8_t CRC = data[0];
	for (uint32_t i = 1; i < g_data_size; ++i)
		CRC += data[i];

	return CRC;
}*/
static uint32_t calculate_CRC(const uint8_t* data) {

	uint32_t CRC = 0;

	for (uint32_t i = 0; i < 32; i += 2) {

		uint16_t value = (data[i] << 8) | data[i + 1];
		if (value % 3 == 0)
			CRC |= 1 << (i + 0);

		if (value % 2 == 0)
			CRC |= 1 << (i + 1);
	}

	return CRC;
}
