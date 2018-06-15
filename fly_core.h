#ifndef __FLY_CORE_H__
#define __FLY_CORE_H__

#include "TXRX_PROTOCOL.h"

extern void fly_core_initialize();
extern void fly_core_process(TXRX::control_data_t* control_data);
extern void fly_core_make_state_data(TXRX::state_data_t* state_data);

#endif /* __FLY_CORE__ */