#ifndef __FLY_CORE_H__
#define __FLY_CORE_H__

#include "TXRX_PROTOCOL.h"

namespace FLY_CORE {

	const uint8_t INTERNAL_CMD_PROCESS = 0;
	const uint8_t INTERNAL_CMD_DISABLE = 1;

	void initialize();
	void process(uint32_t internal_cmd, TXRX::control_data_t* control_data);
	void make_state_data(TXRX::state_data_t* state_data);
}

#endif /* __FLY_CORE__ */