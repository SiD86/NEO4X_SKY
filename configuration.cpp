#include <Arduino.h>
#include "LIBRARY\EEPROM.h"
#include "LIBRARY\USART1_PDC.h"
#include "LED.h"
#include "configuration.h"
#include "communication.h"
#include "TXRX_PROTOCOL.h"
#include "version.h"
#define PACKET_SIZE							(sizeof(TXRX::configuration_data_t))
#define PACKET_SIZE_WITHOUT_CRC				(sizeof(TXRX::configuration_data_t) - sizeof(TXRX::configuration_data_t::CRC))
#define PACKET_DATA_SIZE					(sizeof(TXRX::configuration_data_t::data))

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
bool configuration_reset() {
	return true;
}

bool configuration_load() {

	g_cfg.send_state_interval = 30;			// 30 ms
	g_cfg.communication_break_time = 1000;	// 1000 ms

	g_cfg.angle_protect = 60;				// [-60; 60]
	g_cfg.ESC_PWM_frequency = 400;			// 400 Hz

	g_cfg.LED_disable_time = 1500;			// ms
	g_cfg.LED_enable_time = 50;				// ms

	g_cfg.PID_output_limit = 400;			// 40%
	g_cfg.PID_enable_threshold = 0;			// 0% (enable always)

	g_cfg.PID_X[0] = 0;
	g_cfg.PID_X[1] = 0;
	g_cfg.PID_X[2] = 0;
	g_cfg.I_X_limit = 300;

	g_cfg.PID_Y[0] = 0;
	g_cfg.PID_Y[1] = 0;
	g_cfg.PID_Y[2] = 0;
	g_cfg.I_Y_limit = 300;

	g_cfg.PID_Z[0] = 0;
	g_cfg.PID_Z[1] = 0;
	g_cfg.PID_Z[2] = 0;
	g_cfg.I_Z_limit = 300;
	return true;
}


#define FRAME_ADDRESS			(0)
#define FRAME_COMMAND			(1)
#define FRAME_SIZE_MSB			(2)
#define FRAME_SIZE_LSB			(3)
#define FRAME_DATA				(4)

bool is_recv_request_valid(uint8_t* data, uint32_t size) {

	// Check request size
	if (size != PACKET_SIZE) {
		Serial.println("PACKET_SIZE");
		return false;
	}
	
	TXRX::configuration_data_t* request = (TXRX::configuration_data_t*)data;

	// Check CRC
	uint32_t crc = calculcate_CRC(data);
	if (request->CRC != crc) {
		Serial.println("CRC");
		return false;
	}

	return true;
}


void configuration_enter_to_change_mode() {

	Serial.println("CONFIGURATION");

	uint8_t eeprom_dump[256] = { 0 };
	memset(eeprom_dump, 0xCC, sizeof(eeprom_dump));

	USART1_initialize();

	while (true) {

		// Wait frame
		USART1_start_rx();
		while (USART1_is_frame_receive() == false);

		Serial.println("FRAME RECV");

		// Get frame data and size
		uint8_t* rx_data = USART1_get_rx_buffer_address();
		uint32_t rx_size = USART1_get_frame_size();

		// Check frame
		if (is_recv_request_valid(rx_data, rx_size) == false) {
			Serial.println("PACKET INVALID");
			continue;
		}

		// Process request
		uint8_t* tx_data = USART1_get_tx_buffer_address();
		TXRX::configuration_data_t* request = (TXRX::configuration_data_t*)rx_data;
		TXRX::configuration_data_t* response = (TXRX::configuration_data_t*)tx_data;

		switch (request->command) 
		{
		case TXRX::CMD_CONFIG_READ_INFORMATION:
			response->command = request->command;
			response->data[0] = 0xAA;
			response->data[1] = 0xAA;
			response->data[2] = 0xAA;
			response->data[3] = 0xAA;
			response->data[4] = MAIN_VERSION;
			response->data[5] = SUB_VERSION;
			response->data[6] = AUX_VERSION;

			Serial.println("CMD_CONFIG_READ_INFORMATION");
			break;

		case TXRX::CMD_CONFIG_READ_MEMORY:
			response->command = request->command;
			memcpy(response->data, eeprom_dump, sizeof(request->data));
			
			Serial.println("CMD_CONFIG_READ_MEMORY");
			break;

		case TXRX::CMD_CONFIG_WRITE_MEMORY:
			response->command = request->command;
			memcpy(eeprom_dump, request->data, sizeof(request->data));

			Serial.println("CMD_CONFIG_WRITE_MEMORY");
			break;

		case TXRX::CMD_CONFIG_RESET:
			response->command = request->command;

			response->CRC = calculcate_CRC(tx_data);
			USART1_start_tx(PACKET_SIZE);

			Serial.println("CMD_CONFIG_RESET");
			while (USART1_is_tx_complete() == false);

			REG_RSTC_CR = 0xA5000005;
			continue;

		default:
			Serial.println("UNKNOWN CMD");
			continue;
		}

		response->CRC = calculcate_CRC(tx_data);
		USART1_start_tx(PACKET_SIZE);

		while (USART1_is_tx_complete() == false);
	}

	/*uint8_t memory_dump[256] = { 0 };
	bool is_ready = EEPROM_read_bytes(0x0000, memory_dump, sizeof(memory_dump));

	// Configuration loop
	while (true) {

		// Wait command
		while (CSS::synchronous_process(false, true) == false);

		// Check data
		if (is_ready == false || g_rx_cfg_data.address > 0xFF || g_rx_cfg_data.bytes > sizeof(TXRX::configure_data_t::data)) {
			g_tx_cfg_data.cmd	  = TXRX::CFG_OPERATION_ERROR;
			g_tx_cfg_data.bytes   = TXRX::CFG_OPERATION_ERROR;
			g_tx_cfg_data.address = TXRX::CFG_OPERATION_ERROR;
			memset(g_tx_cfg_data.data, TXRX::CFG_OPERATION_ERROR, sizeof(g_tx_cfg_data.data));
			CSS::synchronous_process(true, false);
			continue;
		}

		// Process command
		switch (g_rx_cfg_data.cmd)
		{
		case TXRX::CFG_CMD_READ_DEVICE_ID:
			g_tx_cfg_data.cmd	  = g_rx_cfg_data.cmd;
			g_tx_cfg_data.bytes	  = g_rx_cfg_data.bytes;
			g_tx_cfg_data.address = g_rx_cfg_data.address;
			g_tx_cfg_data.data[0] = 0x99;
			g_tx_cfg_data.data[1] = 0x88;
			g_tx_cfg_data.data[2] = 0x77;
			g_tx_cfg_data.data[3] = 0x66;
			break;

		case TXRX::CFG_CMD_READ_BLOCK:
			g_tx_cfg_data.cmd     = g_rx_cfg_data.cmd;
			g_tx_cfg_data.bytes   = g_rx_cfg_data.bytes;
			g_tx_cfg_data.address = g_rx_cfg_data.address;
			memcpy(g_tx_cfg_data.data, &memory_dump[g_rx_cfg_data.address], g_rx_cfg_data.bytes);
			break;

		case TXRX::CFG_CMD_WRITE_BLOCK:
			if (EEPROM_write_bytes(g_rx_cfg_data.address, g_rx_cfg_data.data, g_rx_cfg_data.bytes) == false) {
				g_tx_cfg_data.cmd     = TXRX::CFG_OPERATION_ERROR;
				g_tx_cfg_data.bytes   = TXRX::CFG_OPERATION_ERROR;
				g_tx_cfg_data.address = TXRX::CFG_OPERATION_ERROR;
				memset(g_tx_cfg_data.data, TXRX::CFG_OPERATION_ERROR, sizeof(g_tx_cfg_data.data));
			}
			else {
				g_tx_cfg_data.cmd     = g_rx_cfg_data.cmd;
				g_tx_cfg_data.bytes   = g_rx_cfg_data.bytes;
				g_tx_cfg_data.address = g_rx_cfg_data.address;
				memcpy(&memory_dump[g_rx_cfg_data.address], g_rx_cfg_data.data, g_rx_cfg_data.bytes);
			}
			break;

		case TXRX::CFG_CMD_SET_DEFAULT:
			for (int i = 0; i < 256; ++i)
				memory_dump[i] = i;

			if (EEPROM_write_bytes(0x0000, memory_dump, sizeof(memory_dump)) == false) {
				g_tx_cfg_data.cmd     = TXRX::CFG_OPERATION_ERROR;
				g_tx_cfg_data.bytes   = TXRX::CFG_OPERATION_ERROR;
				g_tx_cfg_data.address = TXRX::CFG_OPERATION_ERROR;
				memset(g_tx_cfg_data.data, TXRX::CFG_OPERATION_ERROR, sizeof(g_tx_cfg_data.data));
			}
			else {
				g_tx_cfg_data.cmd     = g_rx_cfg_data.cmd;
				g_tx_cfg_data.bytes   = g_rx_cfg_data.bytes;
				g_tx_cfg_data.address = g_rx_cfg_data.address;
				memset(g_tx_cfg_data.data, 0, sizeof(g_tx_cfg_data.data));
			}
			break;

		case TXRX::CFG_CMD_SOFTWARE_RESET:
			g_tx_cfg_data.cmd     = g_rx_cfg_data.cmd;
			g_tx_cfg_data.bytes   = g_rx_cfg_data.bytes;
			g_tx_cfg_data.address = g_rx_cfg_data.address;
			memset(g_tx_cfg_data.data, 0, sizeof(g_tx_cfg_data.data));

			CSS::synchronous_process(true, false);

			REG_RSTC_CR = 0xA5000005;
			continue;

		default: // Unknown command
			g_tx_cfg_data.cmd     = TXRX::CFG_OPERATION_ERROR;
			g_tx_cfg_data.bytes   = TXRX::CFG_OPERATION_ERROR;
			g_tx_cfg_data.address = TXRX::CFG_OPERATION_ERROR;
			memset(g_tx_cfg_data.data, TXRX::CFG_OPERATION_ERROR, sizeof(g_tx_cfg_data.data));
			break;
		}

		CSS::synchronous_process(true, false);
	}*/
}

static uint32_t calculcate_CRC(uint8_t* data) {

	uint32_t CRC = 0;
	for (uint32_t i = 0; i < PACKET_SIZE_WITHOUT_CRC; ++i) {
		CRC += data[i];
	}
	return CRC;
}