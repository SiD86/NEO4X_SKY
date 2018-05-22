#ifndef __ADDITIONAL_SUBSYSTEM_H__
#define __ADDITIONAL_SUBSYSTEM_H__

#include "TXRX_PROTOCOL.h"

namespace ASS {
	const uint32_t NO_ERROR								= 0x00;
	const uint32_t MAIN_POWER_SUPPLY_LOW_VOLTAGE		= 0x01;
	const uint32_t WIRELESS_POWER_SUPPLY_LOW_VOLTAGE	= 0x02;
	const uint32_t CAMERA_POWER_SUPPLY_LOW_VOLTAGE		= 0x03;
	const uint32_t SENSORS_POWER_SUPPLY_LOW_VOLTAGE		= 0x04;
}

// External interface
namespace ASS {

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
 