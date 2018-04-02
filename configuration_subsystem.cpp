#include <Arduino.h>
#include "LIBRARY\EEPROM.h"
#include "configuration_subsystem.h"
#include "communication_subsystem.h"

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

CONFIGSS::configuration_t g_cfg;


//
// EXTERNAL INTERFACE
//
bool CONFIGSS::reset_configuration() {
	
	/*if (EEPROM_write_4bytes(0x0000, 0x01, 1) == false)	// Memory map version
		return false;
	
	if (EEPROM_write_4bytes(0x0001, 0x00, 1) == false)	// FW version
		return false;
	
	if (EEPROM_write_4bytes(0x0002, 0x78563412, 4) == false) // Device ID
		return false;
	
	if (EEPROM_write_4bytes(0x0030, 0x00, 1) == false)	// Calibration ESC
		return false;
	
	if (EEPROM_write_4bytes(0x0031, 400, 2) == false)	// ESC PWM frequency
		return false;
	
	if (EEPROM_write_4bytes(0x0033, 1000, 2) == false)	// Low battery voltage
		return false;
	
	if (EEPROM_write_4bytes(0x0035, 1000, 2) == false)	// Connection lost timeout
		return false;
	
	if (EEPROM_write_4bytes(0x0037, 100, 1) == false)	// Send state data interval
		return false;
	
	if (EEPROM_write_4bytes(0x0040, 2500, 2) == false)	// PID interval
		return false;
	
	if (EEPROM_write_4bytes(0x0042, 300, 2) == false)	// PID limit
		return false;
	
	if (EEPROM_write_4bytes(0x0044, 0, 2) == false)		// PID threshold
		return false;
	*/
	return true;
}

bool CONFIGSS::load_and_check_configuration() {

	g_cfg.send_state_interval = 30;			// 30 ms
	g_cfg.desync_silence_window_time = 200; // 200 ms (!!! < connection_lost_timeout !!!)
	g_cfg.connection_lost_timeout = 1000;	// 1000 ms

	g_cfg.angle_protect = 60;				// [-60; 60]
	g_cfg.ESC_PWM_frequency = 400;			// 400 Hz

	g_cfg.battery_low_voltage = 100;		// 10.0V

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

	//uint8_t buffer[256] = { 0 };
	/*if (EEPROM_read_bytes(0x0000, buffer, 256) == false)
		return false;*/

	/*memcpy(&g_cfg.memory_map_version,			&buffer[0x0000], 1);
	memcpy(&g_cfg.FW_version,					&buffer[0x0001], 1);
	memcpy(&g_cfg.device_ID,					&buffer[0x0002], 4);

	memcpy(&g_cfg.ESC_PWM_frequency,			&buffer[0x0031], 2);

	memcpy(&g_cfg.connection_lost_timeout,		&buffer[0x0035], 2);
	memcpy(&g_cfg.send_state_packet_interval,	&buffer[0x0035], 2);

	memcpy(&g_cfg.battery_low_voltage,			&buffer[0x0033], 2);

	memcpy(&g_cfg.PID_output_limit,				&buffer[0x0042], 2);
	memcpy(&g_cfg.PID_enable_threshold,			&buffer[0x0044], 2);

	memcpy(&g_cfg.PID_X,						&buffer[0x0050], 12);
	memcpy(&g_cfg.I_X_limit,					&buffer[0x005C], 4);
		
	memcpy(&g_cfg.PID_Y,						&buffer[0x0060], 12);
	memcpy(&g_cfg.I_Y_limit,					&buffer[0x006C], 4);

	memcpy(&g_cfg.PID_Z,						&buffer[0x0070], 12);
	memcpy(&g_cfg.I_Z_limit,					&buffer[0x007C], 4);

	memcpy(&g_cfg.PID_H,						&buffer[0x0080], 12);
	memcpy(&g_cfg.I_H_limit,					&buffer[0x008C], 4);*/
	
	return true;
}

void CONFIGSS::enter_to_configuration_mode() {

	uint8_t memory_dump[256] = { 0 };
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
	}
}

