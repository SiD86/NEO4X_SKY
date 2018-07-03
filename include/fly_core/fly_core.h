#ifndef FLY_CORE_H_
#define FLY_CORE_H_

#include "fly_protocol.h"

extern void fly_core_initialize(void);
extern void fly_core_process(fly_protocol_control_data_t* control_data);
extern void fly_core_make_state_data(fly_protocol_state_data_t* state_data);

#endif /* FLY_CORE_H_ */