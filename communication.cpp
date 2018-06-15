#include <Arduino.h>
#include "LIBRARY\USART3_PDC.h"
#include "communication.h"
#include "configuration.h"
#include "util.h"
#define DESYNC_SILENCE_TIME		(200)	// ms
#define PACKET_SIZE				(sizeof(TXRX::fly_controller_packet_t))
#define PACKET_DATA_SIZE		(sizeof(TXRX::fly_controller_packet_t::data))

static void async_process_rx();
static void async_process_tx();
static bool is_rx_data_valid(uint8_t* data, uint32_t packet_type);
static uint32_t calculate_CRC(const uint8_t* data);

static uint32_t g_status = COMMUNICATION_NO_ERROR;

TXRX::control_data_t g_comm_cp = {0};
TXRX::state_data_t g_comm_sp = {0};


//
// EXTERNAL INTERFACE
//
void communication_initialize() {
	USART3_initialize();
	USART3_RX_start(PACKET_SIZE);
}

void communication_process() {

	if (USART3_is_error() == true)
		USART3_reset(true, true);

	async_process_tx();
	async_process_rx();
}

uint32_t communication_get_status() {
	return g_status;
}


//
// INTERNAL INTERFACE
//
static void async_process_tx() {

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
	memcpy(packet->data, &g_comm_sp, PACKET_DATA_SIZE);
	packet->CRC = calculate_CRC(packet->data);

	// Start transmit
	USART3_TX_start(PACKET_SIZE);

	prev_tx_time = millis();	// Update time 
	++packet_number;			// Update packet counter
}

static void async_process_rx() {

	static uint32_t prev_rx_data_time = 0;
	uint32_t current_time = millis();

	if (USART3_RX_is_complete() == true) {

		// Process data if no desync communication
		if (IS_BIT_CLEAR(g_status, COMMUNICATION_DESYNC)) {

			static uint32_t error_count = 0;

			uint8_t* recv_data = USART3_RX_get_buffer_address();
			if (is_rx_data_valid(recv_data, TXRX::TYPE_CONTROL_PACKET) == true) {

				// Copy data
				TXRX::fly_controller_packet_t* packet = (TXRX::fly_controller_packet_t*)recv_data;
				memcpy(&g_comm_cp, packet->data, PACKET_DATA_SIZE);

				// Reset error counter and connection lost bit
				error_count = 0;
				CLEAR_BIT(g_status, COMMUNICATION_BREAK);
			}
			else {

				if (++error_count >= 5) {
					SET_BIT(g_status, COMMUNICATION_DESYNC);
				}
			}
		}

		// Update time and start receive next data
		prev_rx_data_time = current_time;
		USART3_RX_start(PACKET_SIZE);
	}
	else {

		// Check desync communication
		if (IS_BIT_SET(g_status, COMMUNICATION_DESYNC)) {

			// Wait silence window and reset receiver
			if (current_time - prev_rx_data_time > DESYNC_SILENCE_TIME) {
				USART3_reset(false, true);
				USART3_RX_start(PACKET_SIZE);
				CLEAR_BIT(g_status, COMMUNICATION_DESYNC);
			}
		}

		// Check communication break
		if (current_time - prev_rx_data_time > g_cfg.communication_break_time) {
			SET_BIT(g_status, COMMUNICATION_BREAK);
		}
	}
}

static bool is_rx_data_valid(uint8_t* data, uint32_t packet_type) {

	TXRX::fly_controller_packet_t* packet = (TXRX::fly_controller_packet_t*)data;

	// Check packet type
	if (packet->type != packet_type) {
		return false;
	}

	// Check CRC
	uint32_t CRC = calculate_CRC(packet->data);
	if (packet->CRC != CRC) {
		return false;
	}

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
