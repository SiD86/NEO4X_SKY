#ifndef __ADDITIONAL_SUBSYSTEM_H__
#define __ADDITIONAL_SUBSYSTEM_H__

#include "TXRX_PROTOCOL.h"

namespace ASS {
	const uint32_t NO_ERROR					= 0x00;
	const uint32_t BATTERY_LOW_VOLTAGE		= 0x01;
}

// External interface
namespace ASS {

	void initialize(uint32_t battery_low_voltage);
	void process();
	void make_state_data(TXRX::state_data_t* state_data);
	uint32_t get_status();
}

#endif /* __ADDITIONAL_SUBSYSTEM_H__ */
 