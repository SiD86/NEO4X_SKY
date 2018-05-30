#include <Arduino.h>
#include "LIBRARY\USART3_PDC.h"
#include "communication_subsystem.h"
#include "configuration_subsystem.h"
#include "util.h"

static const uint32_t g_packet_size = sizeof(TXRX::fly_controller_packet_t);
static const uint32_t g_data_size   = sizeof(TXRX::fly_controller_packet_t::data);

static void asynchronous_process_rx();
static void asynchronous_process_tx();
static bool process_rx_data(void* data, uint32_t packet_type);
static uint32_t calculate_CRC(const uint8_t* data);

static uint32_t g_status = CSS::NO_ERROR;

TXRX::control_data_t g_cp = {0};
TXRX::state_data_t g_sp = {0};
TXRX::configure_data_t g_rx_cfg_data = { 0 };
TXRX::configure_data_t g_tx_cfg_data = { 0 };


//
// EXTERNAL INTERFACE
//
void CSS::initialize() {
	USART3_initialize();
	USART3_RX_start(g_packet_size);
}

void CSS::asynchronous_process() {

	if (USART3_is_error() == true)
		USART3_reset(true, true);

	asynchronous_process_tx();
	asynchronous_process_rx();
}

bool CSS::synchronous_process(bool tx, bool rx) {

	if (tx == true) {

		TXRX::fly_controller_packet_t* packet = (TXRX::fly_controller_packet_t*)USART3_TX_get_buffer_address();
		
		// Build new packet
		packet->type = TXRX::TYPE_CONFIG_ACK_PACKET;
		memcpy(packet->data, &g_tx_cfg_data, g_data_size);
		packet->CRC = calculate_CRC(packet->data);

		// Start send packet and wait operation complete
		USART3_TX_start(g_packet_size);
		while (USART3_TX_is_complete() == false) {
			if (USART3_is_error() == true) {
				USART3_reset(true, false);
				return false;
			}
		}
	}
	else {

		// Wait packet
		USART3_RX_start(g_packet_size);
		while (USART3_RX_is_complete() == false) {
			if (USART3_is_error() == true) {
				USART3_reset(false, true);
				return false;
			}
		}

		if (process_rx_data(&g_rx_cfg_data, TXRX::TYPE_CONFIG_REQ_PACKET) == false)
			return false;
	}

	return true;
}

uint32_t CSS::get_status() {
	return g_status;
}


//
// INTERNAL INTERFACE
//
static void asynchronous_process_tx() {

	static uint16_t packet_number = 0;
	static uint32_t prev_tx_time = 0;

	// Check TX interval
	if (millis() - prev_tx_time < g_cfg.send_state_interval)
		return;

	// Check complete TX previus data
	if (USART3_TX_is_complete() == false)
		return;

	// Build new packet
	TXRX::fly_controller_packet_t* packet = (TXRX::fly_controller_packet_t*)USART3_TX_get_buffer_address();
	packet->type = TXRX::TYPE_STATE_PACKET;
	packet->number = packet_number;
	memcpy(packet->data, &g_sp, g_data_size);
	packet->CRC = calculate_CRC(packet->data);

	// Start transmit
	USART3_TX_start(g_packet_size);

	prev_tx_time = millis();	// Update time 
	++packet_number;			// Update packet counter
}

static void asynchronous_process_rx() {

	static uint32_t prev_rx_data_time = 0;
	uint32_t current_time = millis();

	if (USART3_RX_is_complete() == true) {

		// Process data
		if (IS_BIT_CLEAR(g_status, CSS::DESYNC)) {

			static uint32_t error_count = 0;
			if (process_rx_data(&g_cp, TXRX::TYPE_CONTROL_PACKET) == true) {
				error_count = 0;
				CLEAR_STATUS_BIT(g_status, CSS::CONNECTION_LOST);
			}
			else {

				if (++error_count >= 5) {
					SET_STATUS_BIT(g_status, CSS::DESYNC);
				}
			}
		}

		// Update time and initialize start receive next data
		prev_rx_data_time = current_time;
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

static bool process_rx_data(void* data, uint32_t packet_type) {

	TXRX::fly_controller_packet_t* packet = (TXRX::fly_controller_packet_t*)USART3_RX_get_buffer_address();

	// Check packet type
	if (packet->type != packet_type) {
		return false;
	}

	// Check CRC
	uint32_t CRC = calculate_CRC(packet->data);
	if (packet->CRC != CRC) {
		return false;
	}

	// Copy data
	memcpy(data, packet->data, g_data_size);
	return true;
}

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
