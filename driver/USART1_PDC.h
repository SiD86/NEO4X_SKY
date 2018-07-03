#ifndef USART1_PDR_H_
#define USART1_PDR_H_

/*******************************************************************************
* @brief	Initialize USART
* @note		Mode 8N1
*******************************************************************************/
void USART1_initialize();

/*******************************************************************************
* @brief	Reset USART
* @note		Reset status register, reset transmitter and receiver
* @param	tx: true - reset TX
* @param	rx: true - reset RX
*******************************************************************************/
void USART1_reset(bool tx, bool rx);

/*******************************************************************************
* @brief	Asynchronous transmit control
* @param	size: bytes count for transmit
*******************************************************************************/
void USART1_start_tx(uint32_t size);
bool USART1_is_tx_complete();
uint8_t* USART1_get_tx_buffer_address();

/*******************************************************************************
* @brief	Asynchronous receive control
*******************************************************************************/
void USART1_start_rx();
bool USART1_is_frame_receive();
uint32_t USART1_get_frame_size();
uint8_t* USART1_get_rx_buffer_address();

/*******************************************************************************
* @brief	Check USART errors
* @note		Check overrun error, framing error, parity error. Need reset USART
* @retval	true - error, false - no errors
*******************************************************************************/
bool USART1_is_error();

#endif /* USART1_PDR_H_ */
