#include <stdbool.h>
#include <string.h>
#include "sam3x8e.h"
#include "EEPROM.h"
#include "USART1_PDC.h"
#include "LED.h"
#include "configuration.h"
#include "communication.h"
#include "fly_protocol.h"
#include "version.h"
#define PACKET_SIZE							(sizeof(wire_packet_t))
#define PACKET_SIZE_WITHOUT_CRC				(PACKET_SIZE - 4)
#define PACKET_DATA_SIZE					(256)

#define EE_MEMORY_MAP_VERSION				(0x0000)
#define EE_FW_VERSION						(0x0002)
#define EE_DEVICE_ID						(0x0004)
#define EE_CONNECTION_TIMEOUT				(0x0020)
#define EE_SEND_STATE_PACKET_INTERVAL		(0x0022)
#define EE_ESC_FREQUENCY					(0x0030)
#define EE_BATTERY_LOW_VOLTAGE				(0x0034)

#define EE_PID_OUTPUT_LIMIT					(0x0040)
#define EE_PID_ENABLE_THRESHOLD				(0x0042)
#define	EE_AXIS_X_BASE_ADDRESS				(0x0050)
#define	EE_AXIS_Y_BASE_ADDRESS				(0x0060)
#define	EE_AXIS_Z_BASE_ADDRESS				(0x0070)
#define	EE_AXIS_H_BASE_ADDRESS				(0x0080)
#define PID_P_OFFSET						(0x0000)
#define PID_I_OFFSET						(0x0004)
#define PID_D_OFFSET						(0x0008)
#define PID_I_LIMIT_OFFSET					(0x000C)

configuration_t g_cfg;

static uint32_t calculcate_CRC(uint8_t* data);

//
// EXTERNAL INTERFACE
//
bool configuration_reset(void) {
	return true;
}

bool configuration_load(void) {
	
	g_cfg.send_state_interval = 30;			// 30 ms
	g_cfg.communication_break_time = 1000;	// 1000 ms

	g_cfg.angle_protect = 60;				// [-60; 60]
	g_cfg.ESC_PWM_frequency = 400;			// 400 Hz

	g_cfg.led_disable_time = 1500;			// ms
	g_cfg.led_enable_time = 50;				// ms

	g_cfg.pid_output_limit = 400;			// 40%
	g_cfg.pid_enable_threshold = 0;			// 0% (enable always)

	g_cfg.rate_pid_x[0] = 0;
	g_cfg.rate_pid_x[1] = 0;
	g_cfg.rate_pid_x[2] = 0;
	g_cfg.rate_i_x_limit = 300;
	
	g_cfg.rate_pid_y[0] = 0;
	g_cfg.rate_pid_y[1] = 0;
	g_cfg.rate_pid_y[2] = 0;
	g_cfg.rate_i_y_limit = 300;
	
	g_cfg.rate_pid_z[0] = 0;
	g_cfg.rate_pid_z[1] = 0;
	g_cfg.rate_pid_z[2] = 0;
	g_cfg.rate_i_z_limit = 300;
	
	g_cfg.angle_pid_x[0] = 0;
	g_cfg.angle_pid_x[1] = 0;
	g_cfg.angle_pid_x[2] = 0;
	g_cfg.angle_i_x_limit = 300;
	
	g_cfg.angle_pid_y[0] = 0;
	g_cfg.angle_pid_y[1] = 0;
	g_cfg.angle_pid_y[2] = 0;
	g_cfg.angle_i_y_limit = 300;
	
	g_cfg.angle_pid_z[0] = 0;
	g_cfg.angle_pid_z[1] = 0;
	g_cfg.angle_pid_z[2] = 0;
	g_cfg.angle_i_z_limit = 300;
	
	return true;
}

bool is_recv_request_valid(uint8_t* data, uint32_t size) {

	// Check request size
	if (size != PACKET_SIZE) {
		return false;
	}
	
	wire_packet_t* request = (wire_packet_t*)data;

	// Check CRC
	uint32_t crc = calculcate_CRC(data);
	if (request->CRC != crc) {
		return false;
	}

	return true;
}

void configuration_enter_to_change_mode(void) {

	// Read memory
	uint8_t eeprom_dump[256] = { 0 };
	EEPROM_read_bytes(0x0000, eeprom_dump, 256);

	USART1_initialize();

	while (true) {

		// Wait frame
		USART1_start_rx();
		while (USART1_is_frame_receive() == false);

		// Get frame data and size
		uint8_t* rx_data = USART1_get_rx_buffer_address();
		uint32_t rx_size = USART1_get_frame_size();

		// Check frame
		if (is_recv_request_valid(rx_data, rx_size) == false) {
			continue;
		}

		// Process request
		uint8_t* tx_data = USART1_get_tx_buffer_address();
		wire_packet_t* request  = (wire_packet_t*)rx_data;
		wire_packet_t* response = (wire_packet_t*)tx_data;

		switch (request->command) 
		{
		case WIRE_CMD_CONFIG_READ_INFORMATION:
			response->command = request->command;
			response->data[0] = 0xAA;
			response->data[1] = 0xAA;
			response->data[2] = 0xAA;
			response->data[3] = 0xAA;
			response->data[4] = MAIN_VERSION;
			response->data[5] = SUB_VERSION;
			response->data[6] = AUX_VERSION;
			break;

		case WIRE_CMD_CONFIG_READ_MEMORY:
			response->command = request->command;
			memcpy(response->data, eeprom_dump, sizeof(request->data));
			break;

		case WIRE_CMD_CONFIG_WRITE_MEMORY:
			response->command = request->command;
			memcpy(eeprom_dump, request->data, sizeof(request->data));
			EEPROM_write_bytes(0x0000, eeprom_dump, 256);
			break;

		case WIRE_CMD_CONFIG_RESET:
			response->command = request->command;
			response->CRC = calculcate_CRC(tx_data);

			USART1_start_tx(PACKET_SIZE);
			while (USART1_is_tx_complete() == false);

			REG_RSTC_CR = 0xA5000005;
			continue;

		default:
			continue;
		}

		response->CRC = calculcate_CRC(tx_data);
		USART1_start_tx(PACKET_SIZE);

		while (USART1_is_tx_complete() == false);
	}
}

static uint32_t calculcate_CRC(uint8_t* data) {

	uint32_t CRC = 0;
	for (uint32_t i = 0; i < PACKET_SIZE_WITHOUT_CRC; ++i) {
		CRC += data[i];
	}
	return CRC;
}