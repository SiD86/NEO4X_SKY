#ifndef __EEPROM_H__
#define __EEPROM_H__

// LIBRARY VERSION: 0.0.0

/**************************************************************************
* @brief	Function for read data from EEPROM
* @param	address: address to EEPROM memory cell
* @param	data: buffer address
* @param	size: count bytes for read
* @retval	true - read success, false - fail
**************************************************************************/
bool EEPROM_read_bytes(uint32_t address, uint8_t* data, uint32_t size);

/**************************************************************************
* @brief	Function for write data to EEPROM
* @param	address: address to EEPROM memory cell
* @param	data: buffer address
* @param	size: count bytes for write
* @retval	true - read success, false - fail
**************************************************************************/
bool EEPROM_write_bytes(uint32_t address, uint8_t* data, uint32_t size);
bool EEPROM_write_4bytes(uint32_t address, uint32_t data, uint32_t size);

#endif /* __EEPROM_H__ */