#pragma once
////////////////////////////////////////////////////////
//
// VERSION: 0.0.4
//
////////////////////////////////////////////////////////

/*******************************************************************************
* @brief	Initialize USART
* @note		Mode 8N1
* @param	speed: USART baudrate or SPI speed
*******************************************************************************/
void USART3_initialize(uint32_t speed);

/*******************************************************************************
* @brief	Reset USART
* @note		Reset status register, reset transmitter and receiver
* @param	tx: true - reset TX
* @param	rx: true - reset RX
*******************************************************************************/
void USART3_reset(bool tx, bool rx);

/*******************************************************************************
* @brief	Asynchronous transmit control
* @param	size: bytes count for transmit
*******************************************************************************/
void USART3_TX_start(uint32_t size);
bool USART3_TX_is_complete();
uint8_t* USART3_TX_get_buffer_address();

/*******************************************************************************
* @brief	Asynchronous receive control
* @param	size: bytes count for receive
*******************************************************************************/
void USART3_RX_start(uint32_t size);
bool USART3_RX_is_complete();
uint8_t* USART3_RX_get_buffer_address();

/*******************************************************************************
* @brief	Check USART errors
* @note		Cehck overrun error, framing error, parity error. Need reset USART
* @retval	true - error, false - no errors
*******************************************************************************/
bool USART3_is_error();
