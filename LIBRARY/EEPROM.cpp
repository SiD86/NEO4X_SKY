#include <Arduino.h>
#include "I2C.h"
#include "EEPROM.h"
#define ADDRESS						(0x50)
#define MAX_BLOCK_SIZE				(7)

bool EEPROM_read_bytes(uint32_t address, uint8_t* data, uint32_t size) {

	I2C_set_internal_address_length(2);

	while (size != 0) {

		// Read block from EEPROM
		uint32_t count = min(size, MAX_BLOCK_SIZE);
		if (I2C_read_bytes(ADDRESS, address, data, count) == false)
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

		// Write block to EEPROM
		uint32_t count = min(size, MAX_BLOCK_SIZE);
		if (I2C_write_bytes(ADDRESS, address, data, count) == false)
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