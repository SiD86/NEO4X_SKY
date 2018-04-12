#ifndef __ADDITIONAL_SUBSYSTEM_H__
#define __ADDITIONAL_SUBSYSTEM_H__

#include "TXRX_PROTOCOL.h"

namespace ASS {
	const uint32_t NO_ERROR					= 0x00;
	const uint32_t BATTERY_LOW_VOLTAGE		= 0x01;
}

// External interface
namespace ASS {

	/**************************************************************************
	* @brief	Function for initialize subsystem
	**************************************************************************/
	void initialize();

	/**************************************************************************
	* @brief	Function for process subsystem
	* @note		Measurement voltage, temperature, vibration 
	**************************************************************************/
	void process();

	/**************************************************************************
	* @brief	Function for load subsystem data to state packet
	* @param	state_data: state packet address
	**************************************************************************/
	void make_state_data(TXRX::state_data_t* state_data);

	/**************************************************************************
	* @brief	Function for get subsystem status
	* @retval	Status
	**************************************************************************/
	uint32_t get_status();
}

#endif /* __ADDITIONAL_SUBSYSTEM_H__ */
 