#include <Arduino.h>
#include "TXRX_PROTOCOL.h"
#include "LIBRARY\I2C.h"
#include "configuration_subsystem.h"
#define EEPROM_ADDRESS						(0x50)

static bool read_memory(uint8_t* buffer);
static bool write_memory(uint8_t* buffer);
static bool read_block_32bytes(uint32_t address, uint8_t* buffer);
static bool write_block_32bytes(uint32_t address, uint8_t* buffer);
static bool check_configuration();

CONFIGSS::configuration_t g_configuration;


//
// EXTERNAL INTERFACE
//
bool CONFIGSS::load_and_check_configuration() {

	uint8_t buffer[256] = { 0 };
	if (read_memory(buffer) == false)
		return false;

	memcpy(&g_configuration.memory_map_version, &buffer[0x0000], 1);
	memcpy(&g_configuration.firmware_version,	&buffer[0x0001], 1);
	memcpy(&g_configuration.device_ID,			&buffer[0x0002], 4);

	memcpy(&g_configuration.calibration_ESC,	&buffer[0x0030], 1);
	memcpy(&g_configuration.PWM_frequency_ESC,	&buffer[0x0031], 2);

	memcpy(&g_configuration.battery_low_voltage,&buffer[0x0033], 2);
	memcpy(&g_configuration.connection_timeout, &buffer[0x0035], 2);

	memcpy(&g_configuration.PID_interval,		&buffer[0x0040], 2);
	memcpy(&g_configuration.PID_limit,			&buffer[0x0042], 2);
	memcpy(&g_configuration.PID_start,			&buffer[0x0044], 2);

	memcpy(&g_configuration.PID_X,				&buffer[0x0050], 12);
	memcpy(&g_configuration.I_X_limit,			&buffer[0x005C], 4);
	
	memcpy(&g_configuration.PID_Y,				&buffer[0x0060], 12);
	memcpy(&g_configuration.I_Y_limit,			&buffer[0x006C], 4);

	memcpy(&g_configuration.PID_Z,				&buffer[0x0070], 12);
	memcpy(&g_configuration.I_Z_limit,			&buffer[0x007C], 4);

	memcpy(&g_configuration.PID_H,				&buffer[0x0080], 12);
	memcpy(&g_configuration.I_H_limit,			&buffer[0x008C], 4);

	// Reset calibration ESC parameter
	if (g_configuration.calibration_ESC == 0x01) {
		buffer[0x0030] = 0x00;
		write_memory(buffer);
	}
	
	return check_configuration();
}

void CONFIGSS::enter_to_configuration_mode() {

	uint8_t buffer[256] = { 0 };
	bool is_need_reset_UART = false;

	// Configuration loop
	while (true) {
		if (Serial.available() > 3 || is_need_reset_UART == true) {
			Serial.end();
			while (Serial.available())
				Serial.read();
			Serial.begin(460800);
			delay(100);
			continue;
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

		case TXRX::UART_CMD_CHECK_MODE:
			data_for_write = TXRX::UART_ACK_SUCCESS;
			Serial.write(&data_for_write, 1);
			break;

		case TXRX::UART_CMD_EXIT:
			data_for_write = TXRX::UART_ACK_SUCCESS;
			Serial.write(&data_for_write, 1);
			return;

		case TXRX::UART_CMD_GET_MEMORY_BLOCK:
			Serial.write(&buffer[arg1 * 32], 32);
			break;

		case TXRX::UART_CMD_WRITE_CELL:
			buffer[arg1] = arg2;
			data_for_write = TXRX::UART_ACK_SUCCESS;
			Serial.write(&data_for_write, 1);
			break;

		case TXRX::UART_CMD_LOAD_MEMORY_PAGE:
			data_for_write = TXRX::UART_ACK_SUCCESS;
			if (read_memory(buffer) == false)
				data_for_write = TXRX::UART_ACK_FAIL;
			Serial.write(&data_for_write, 1);
			break;

		case TXRX::UART_CMD_SAVE_MEMORY_PAGE:
			data_for_write = TXRX::UART_ACK_SUCCESS;
			if (write_memory(buffer) == false)
				data_for_write = TXRX::UART_ACK_FAIL;
			Serial.write(&data_for_write, 1);
			break;

		default:
			is_need_reset_UART = true;
			break;
		}
	}
}


//
// INTERNAL INTERFACE
//
static bool read_memory(uint8_t* buffer) {

	I2C_set_internal_address_length(2);

	for (uint32_t address = 0x00; address <= 0xE0; address += 0x20) {

		if (read_block_32bytes(address, buffer + address) == false)
			return false;
	}
	return true;
}

static bool write_memory(uint8_t* buffer) {

	I2C_set_internal_address_length(2);

	for (uint32_t address = 0x00; address <= 0xE0; address += 0x20) {

		if (write_block_32bytes(address, buffer + address) == false)
			return false;
	}
	delay(1000);
	return true;
}

static bool read_block_32bytes(uint32_t address, uint8_t* buffer) {

	for (uint32_t i = 0; i < 10; ++i) {

		if (I2C_read_bytes(EEPROM_ADDRESS, address, buffer, 32) == true)
			return true;
		delay(100);
	}
	return false;
}

static bool write_block_32bytes(uint32_t address, uint8_t* buffer) {

	for (uint32_t i = 0; i < 10; ++i) {

		if (I2C_write_bytes(EEPROM_ADDRESS, address, buffer, 32) == true)
			return true;
		delay(100);
	}
	return false;
}

static bool check_configuration() {

	if (g_configuration.PWM_frequency_ESC > 400) {
		g_configuration.PWM_frequency_ESC = 50;
		return false;
	}
	if (g_configuration.PWM_frequency_ESC < 50) {
		g_configuration.PWM_frequency_ESC = 50;
		return false;
	}

	if (g_configuration.PID_interval == 0) {
		g_configuration.PID_interval = 2500;
		return false;
	}

	return true;
}
