#include <Arduino.h>
#include "LIBRARY\USART3_PDC.h"
#include "TXRX_PROTOCOL.h"
#include "communication_subsystem.h"
#include "util.h"
#define USART_BAUDRATE						(175000)
#define DEVICE_ADDRESS_DEFAULT				(255)
#define CONNECTION_LOST_TIMEOUT_MS			(1000)
#define CONNECTION_SILENCE_DELAY_MS			(200)
#define SEND_PACKET_INTERVAL				(100)

static const uint32_t g_packet_size = sizeof(TXRX::fly_controller_packet_t);
static const uint32_t g_data_size = sizeof(TXRX::fly_controller_packet_t::data);

static void process_rx();
static void process_tx();
static bool process_rx_data();
static uint8_t calculate_CRC8(const uint8_t* data);

static uint32_t g_status = CSS::NO_ERROR;

TXRX::control_data_t g_cp = {0};
TXRX::state_data_t g_sp = {0};


uint32_t g_hardware_error_count = 0;
uint32_t g_software_error_count = 0;
uint32_t g_desync_count = 0;

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

		Serial.println("USART_ERROR");
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
	if (millis() - prev_tx_time < SEND_PACKET_INTERVAL)
		return;

	// Check complite TX previus data
	if (USART3_TX_is_complete() == false)
		return;

	// Build new packet
	TXRX::fly_controller_packet_t* packet = (TXRX::fly_controller_packet_t*)USART3_TX_get_buffer_address();
	packet->dev_addr = DEVICE_ADDRESS_DEFAULT;
	memcpy(packet->data, &g_sp, g_data_size);
	packet->CRC = calculate_CRC8(packet->data);

	// Start transmit
	USART3_TX_start(g_packet_size);

	// Update time 
	prev_tx_time = millis();
}

static void process_rx() {

	static uint32_t prev_rx_any_data_time = 0;
	static uint32_t prev_rx_state_data_time = 0;

	if (USART3_RX_is_complete() == true) {

		// Update time
		prev_rx_any_data_time = millis();

		// Process data
		if ((g_status & CSS::DESYNC) == 0) {

			static uint32_t error_count = 0;
			if (process_rx_data() == false) {

				++g_software_error_count; // DEBUG
				if (++error_count >= 5) {
					SET_STATUS_BIT(g_status, CSS::DESYNC);
					++g_desync_count; // DEBUG
					Serial.println("DESYNC"); // DEBUG
				}
			}
			else {
				error_count = 0;
				CLEAR_STATUS_BIT(g_status, CSS::CONNECTION_LOST);
				prev_rx_state_data_time = prev_rx_any_data_time;
			}
		}

		// Initialize start receive next data
		USART3_RX_start(g_packet_size);
	}
	else {

		// Check desync communication
		if (g_status & CSS::DESYNC) {

			// Wait silence window and reset receiver
			if (millis() - prev_rx_any_data_time > CONNECTION_SILENCE_DELAY_MS) {
				USART3_reset(false, true);
				USART3_RX_start(g_packet_size);
				CLEAR_STATUS_BIT(g_status, CSS::DESYNC);
				Serial.println("RX RST"); // DEBUG
			}
		}

		// Check communication timeout
		if (millis() - prev_rx_state_data_time > CONNECTION_LOST_TIMEOUT_MS) {
			SET_STATUS_BIT(g_status, CSS::CONNECTION_LOST);
			Serial.println("CONN LOST"); // DEBUG
		}
	}
}

static bool process_rx_data() {

	// Get RX buffer address
	TXRX::fly_controller_packet_t* packet = (TXRX::fly_controller_packet_t*)USART3_RX_get_buffer_address();

	// Verify packet
	uint8_t CRC = calculate_CRC8(packet->data);
	bool is_valid = (packet->dev_addr == DEVICE_ADDRESS_DEFAULT) && (packet->CRC == CRC);
	if (is_valid == true) {
		memcpy(&g_cp, packet->data, g_data_size); // Copy data
		return true;
	}
	return false;
}

static uint8_t calculate_CRC8(const uint8_t* data) {

	uint8_t CRC = data[0];

	for (uint32_t i = 1; i < g_data_size; ++i)
		CRC += data[i];

	return CRC;
}