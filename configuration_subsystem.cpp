#include <Arduino.h>
#include "TXRX_PROTOCOL.h"
#include "LIBRARY\EEPROM.h"
#include "configuration_subsystem.h"

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


static bool check_configuration();

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
	
	return check_configuration();
}

void CONFIGSS::enter_to_configuration_mode() {

	/*bool is_need_reset_UART = false;
	uint8_t memory_image[256] = { 0 };

	EEPROM_read_bytes(0x0000, memory_image, 256);

	// Configuration loop
	while (true) {

		if (Serial.available() > 3 || is_need_reset_UART == true) {
			Serial.end();
			Serial.begin(460800);
			is_need_reset_UART = false;
		}

		if (Serial.available() != 3)
			continue;

		uint8_t cmd = Serial.read();
		uint8_t arg1 = Serial.read();
		uint8_t arg2 = Serial.read();

		uint8_t data_for_write = 0;

		switch (cmd)
		{
		case TXRX::UART_CMD_NO_COMMAND:
			break;

		case TXRX::UART_CMD_WHO_I_AM:
			data_for_write = TXRX::UART_ACK_QUADCOPTER;
			Serial.write(&data_for_write, 1);
			digitalWrite(13, HIGH);
			break;

		case TXRX::UART_CMD_GET_MEMORY_BLOCK:
			Serial.write(&memory_image[arg1 * 32], 32);
			break;

		case TXRX::UART_CMD_WRITE_CELL:
			EEPROM_write_4bytes(arg1, arg2, 1);
			EEPROM_read_bytes(arg1, &memory_image[arg1], 1);

			data_for_write = TXRX::UART_ACK_SUCCESS;
			if (memory_image[arg1] != arg2)
				data_for_write = TXRX::UART_ACK_FAIL;

			Serial.write(&data_for_write, 1);
			break;

		default:
			is_need_reset_UART = true;
			break;
		}
	}*/
}


//
// INTERNAL INTERFACE
//
static bool check_configuration() {

	/*if (g_cfg.PWM_frequency_ESC > 400) {
		g_cfg.PWM_frequency_ESC = 50;
		return false;
	}
	if (g_cfg.PWM_frequency_ESC < 50) {
		g_cfg.PWM_frequency_ESC = 50;
		return false;
	}

	if (g_cfg.PID_interval == 0) {
		g_cfg.PID_interval = 2500;
		return false;
	}*/

	return true;
}
