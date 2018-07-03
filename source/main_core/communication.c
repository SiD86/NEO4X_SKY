#include <stdbool.h>
#include <string.h>
#include "sam3x8e.h"
#include "systimer.h"
#include "USART3_PDC.h"
#include "communication.h"
#include "configuration.h"
#include "utils.h"
#include "UART.h"
#define DESYNC_SILENCE_TIME		(200)	// ms
#define PACKET_SIZE				(sizeof(fly_protocol_packet_t))
#define PACKET_DATA_SIZE		(FLY_PROTOCOL_DATA_SIZE)

static void async_process_rx();
static void async_process_tx();
static bool is_rx_data_valid(uint8_t* data, uint32_t data_size);
static uint32_t calculate_CRC(const uint8_t* data);

static uint32_t g_status = COMMUNICATION_NO_ERROR;

fly_protocol_control_data_t g_comm_cp = {0};
fly_protocol_state_data_t g_comm_sp = {0};


//
// EXTERNAL INTERFACE
//
void communication_initialize() {
	USART3_initialize();
	USART3_start_rx();
}

void communication_process() {

	if (USART3_is_error() == true) {
		USART3_reset(true, true);
		USART3_start_rx();
	}

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

	// Check complete TX previous data
	if (USART3_is_tx_complete() == false)
		return;

	// Build new packet
	fly_protocol_packet_t* packet = (fly_protocol_packet_t*)USART3_get_tx_buffer_address();
	packet->number = packet_number;
	memcpy(packet->data, &g_comm_sp, PACKET_DATA_SIZE);
	packet->CRC = calculate_CRC(packet->data);

	// Start transmit
	USART3_start_tx(PACKET_SIZE);

	prev_tx_time = millis();	// Update time 
	++packet_number;			// Update packet counter
}

static void async_process_rx() {

	static uint32_t prev_rx_data_time = 0;

	uint32_t current_time = millis();

	if (USART3_is_frame_received() == true) {

		uint8_t* recv_data = USART3_get_rx_buffer_address();
		uint32_t data_size = USART3_get_frame_size();
		if (is_rx_data_valid(recv_data, data_size) == true) {

			// Copy packet data field
			fly_protocol_packet_t* packet = (fly_protocol_packet_t*)recv_data;
			memcpy(&g_comm_cp, packet->data, PACKET_DATA_SIZE);

			// Reset connection lost bit
			CLEAR_BIT(g_status, COMMUNICATION_BREAK);
		}

		// Update time and start receive next frame
		prev_rx_data_time = current_time;
		USART3_start_rx();
	}
	else {

		// Check communication break
		if (current_time - prev_rx_data_time > g_cfg.communication_break_time) {
			SET_BIT(g_status, COMMUNICATION_BREAK);
		}
	}
}

static bool is_rx_data_valid(uint8_t* data, uint32_t data_size) {

	fly_protocol_packet_t* packet = (fly_protocol_packet_t*)data;

	// Check packet size
	if (data_size != PACKET_SIZE) {
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
