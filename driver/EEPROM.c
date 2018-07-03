#include <stdbool.h>
#include <string.h>
#include "sam3x8e.h"
#include "I2C.h"
#include "EEPROM.h"
#include "systimer.h"
#define EEPROM_I2C_ADDRESS			(0x50)
#define MAX_BLOCK_SIZE				(7)

bool EEPROM_read_bytes(uint32_t address, uint8_t* data, uint32_t size) {

	I2C_set_internal_address_length(2);

	while (size != 0) {

		// Make block size
		uint32_t count = 0;
		if (size < MAX_BLOCK_SIZE) {
			count = size;
		}
		else {
			count = MAX_BLOCK_SIZE;
		}

		// Read block from EEPROM
		if (I2C_read_bytes(EEPROM_I2C_ADDRESS, address, data, count) == false)
			return false;

		// Offset
		address += count;
		data += count;
		size -= count;
	}
	return true;
}

bool EEPROM_write_bytes(uint32_t address, uint8_t* data, uint32_t size) {

	I2C_set_internal_address_length(2);

	while (size != 0) {

		// Make block size
		uint32_t count = 0;
		if (size < MAX_BLOCK_SIZE) {
			count = size;
		}
		else {
			count = MAX_BLOCK_SIZE;
		}
		
		// Write block to EEPROM
		if (I2C_write_bytes(EEPROM_I2C_ADDRESS, address, data, count) == false)
			return false;

		// Offset
		address += count;
		data += count;
		size -= count;

		delay(50);
	}
	return true;
}

bool EEPROM_write_4bytes(uint32_t address, uint32_t data, uint32_t size) {
	return EEPROM_write_bytes(address, (uint8_t*)&data, size);
}