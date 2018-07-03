#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include "fly_protocol.h"

//
// Error bits
//
#define COMMUNICATION_NO_ERROR				(0x00)
#define COMMUNICATION_BREAK					(0x01)


//
// External interface
//

/**************************************************************************
* @brief	Function for initialize subsystem
**************************************************************************/
extern void communication_initialize();

/**************************************************************************
* @brief	Function for asynchonous communication 
* @note		Handling TX and RX data
* @note		Use only normal mode
**************************************************************************/
extern void communication_process();

/**************************************************************************
* @brief	Function for get subsystem status
* @retval	Status
**************************************************************************/
extern uint32_t communication_get_status();

extern fly_protocol_control_data_t g_comm_cp;
extern fly_protocol_state_data_t g_comm_sp;

#endif /* __COMMUNICATION_H__ */