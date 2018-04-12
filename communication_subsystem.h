#ifndef __COMMUNICATION_SUBSYSTEM_H__
#define __COMMUNICATION_SUBSYSTEM_H__

#include "TXRX_PROTOCOL.h"

// CSS - Communication SubSystem

// Error codes (bitfield)
namespace CSS {
	const uint32_t NO_ERROR					= 0x00;
	const uint32_t DESYNC					= 0x01;
	const uint32_t CONNECTION_LOST			= 0x02;
}

// External interface
namespace CSS {

	/**************************************************************************
	* @brief	Function for initialize subsystem
	**************************************************************************/
	void initialize();

	/**************************************************************************
	* @brief	Function for asynchonous communication 
	* @note		Handling TX and RX data
	* @note		Use only normal mode
	**************************************************************************/
	void asynchronous_process();

	/**************************************************************************
	* @brief	Function for synchonous communication
	* @note		Use only configuration mode. 
	* @note		Error status not used, get_status() function not used
	* @note		tx = true and rx = true - this error
	* @param	tx: true - if need transmit data
	* @param	rx: true - if need receive data
	* @retval	true - if process success, false - timeout or packet damage
	**************************************************************************/
	bool synchronous_process(bool tx, bool rx);

	/**************************************************************************
	* @brief	Function for get subsystem status
	* @retval	Status
	**************************************************************************/
	uint32_t get_status();
}

extern TXRX::control_data_t g_cp;
extern TXRX::state_data_t g_sp;
extern TXRX::configure_data_t g_rx_cfg_data;
extern TXRX::configure_data_t g_tx_cfg_data;

#endif /* __COMMUNICATION_SUBSYSTEM_H__ */