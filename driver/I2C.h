#ifndef I2C_H_
#define I2C_H_

#define I2C_SPEED_100KHZ				(0x1D0D0)
#define I2C_SPEED_400KHZ				(0x6565)

#define I2C_DRIVER_NO_ERROR				(0x00)
#define I2C_DRIVER_BUSY					(0x01)
#define I2C_DRIVER_ERROR				(0x02)

/**************************************************************************
* @brief	Function for initialize GPIO and I2C registers in Master Mode
* @param	clock_speed: I2C_SPEED_100KHZ or I2C_SPEED_400KHZ constants
**************************************************************************/
void I2C_initialize(uint32_t clock_speed);

/**************************************************************************
* @brief	Function for set internal address length
* @param	length: 0 - no internal address, [1,2,3] byte address length
**************************************************************************/
void I2C_set_internal_address_length(uint32_t length);

/**************************************************************************
* @brief	Function for get or reset (only error) current driver state
* @retval	Current driver status
**************************************************************************/
uint32_t I2C_get_status(void);
void I2C_force_reset_error_status(void);

/**************************************************************************
* @brief	Function for synchronous mode write data
* @note		This wrappers for asynchronous mode functions
* @param	dev_addr: I2C device address
* @param	reg_addr: internal register address on I2C device
* @param	data: pointer to data for write
* @param	size: count bytes for write
* @retval	true - no error, false - error
**************************************************************************/
bool I2C_write_byte(uint8_t dev_addr, uint32_t internal_addr, uint8_t data);
bool I2C_write_bytes(uint8_t dev_addr, uint32_t internal_addr, uint8_t* data, uint32_t size);

/**************************************************************************
* @brief	Function for asynchronous mode write data
* @note		This functions using PDC
* @param	dev_addr: I2C device address
* @param	reg_addr: internal register address on I2C device
* @param	data: pointer to data for write
* @param	size: count bytes for write
* @retval	true - operation start success, , false - operation start fail
**************************************************************************/
bool I2C_async_write_bytes(uint8_t dev_addr, uint32_t internal_addr, uint32_t size);
uint8_t* I2C_async_get_tx_buffer_address(void);

/**************************************************************************
* @brief	Function for synchronous mode read data
* @note		This wrappers for asynchronous mode functions
* @param	dev_addr: I2C device address
* @param	reg_addr: internal register address on I2C device
* @param	data: pointer to receive buffer
* @param	size: count bytes for read
* @retval	true - no error, false - error
**************************************************************************/
bool I2C_read_byte(uint8_t dev_addr, uint32_t internal_addr, uint8_t* data);
bool I2C_read_bytes(uint8_t dev_addr, uint32_t internal_addr, uint8_t* data, uint32_t size);

/**************************************************************************
* @brief	Function for asynchronous mode read data
* @note		This functions using PDC
* @param	dev_addr: I2C device address
* @param	reg_addr: internal register address on I2C device
* @param	data: pointer to receive buffer
* @param	size: count bytes for read
* @retval	true - operation start success, false - operation start fail
**************************************************************************/
bool I2C_async_read_bytes(uint8_t dev_addr, uint32_t internal_addr, uint32_t size);
uint8_t* I2C_async_get_rx_buffer_address(void);

/**************************************************************************
* @brief	Function for write bits to register
* @param	dev_addr: I2C device address
* @param	reg_addr: internal register address on I2C device
* @param	mask: bits mask
* @param	bits: bits for written
* @param	timeout: timeout [ms]
* @retval	I2C_TIMEOUT or I2C_NO_ERROR
**************************************************************************/
uint32_t I2C_write_bits(uint8_t dev_addr, uint8_t internal_addr, uint8_t mask, uint8_t bits);

#endif /* I2C_H_ */