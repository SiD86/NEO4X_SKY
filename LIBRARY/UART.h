#ifndef __UART_H__
#define __UART_H__

// LIBRARY VERSION: 0.0.0

/*******************************************************************************
* @brief	Initialize UART
* @note		Mode 8N1
* @param	speed: USART baudrate speed
*******************************************************************************/
void UART_initialize(uint32_t speed);

/*******************************************************************************
* @brief	Enable or disable UART
* @param	is_enable: UART enable
*******************************************************************************/
void UART_set_state(bool is_enable);

/*******************************************************************************
* @brief	Reset transmitter and receiver
*******************************************************************************/
void UART_reset();

/*******************************************************************************
* @brief	Synchronous transmit data
* @param	data: data for transmit
* @param	size: bytes count for transmit
* @param	timeout: operation timeout value
* @retval	true - success, false - timeout
*******************************************************************************/
bool UART_write(uint8_t* data, uint32_t size, uint32_t timeout);
void UART_dbg(const char* msg);

/*******************************************************************************
* @brief	Read data from input buffer
* @param	data: pointer to buffer
* @param	size: bytes count for read
* @param	timeout: operation timeout value
* @retval	true - success, false - size > available bytes
*******************************************************************************/
bool UART_read(uint8_t* data, uint32_t size, uint32_t timeout);

#endif /* __UART_H__ */