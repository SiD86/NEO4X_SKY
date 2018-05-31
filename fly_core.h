#ifndef __FLY_CORE_H__
#define __FLY_CORE_H__

#include "TXRX_PROTOCOL.h"

namespace FLY_CORE {

	void initialize();
	void process(TXRX::control_data_t* control_data);
	void make_state_data(TXRX::state_data_t* state_data);
}

#endif /* __FLY_CORE__ */