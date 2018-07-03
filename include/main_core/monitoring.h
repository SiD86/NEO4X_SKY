#ifndef __MONITORING_H__
#define __MONITORING_H__

#include "fly_protocol.h"

//
// Error bits
//
#define MONITORING_NO_ERROR						(0x00)
#define MONITORING_MAIN_LOW_VOLTAGE				(0x01)
#define MONITORING_WIRELESS_LOW_VOLTAGE			(0x02)
#define MONITORING_CAMERA_LOW_VOLTAGE			(0x03)
#define MONITORING_SENSORS_LOW_VOLTAGE			(0x04)

//
// External interface
//

/**************************************************************************
* @brief	Function for process subsystem
* @note		Measurement voltage, temperature, vibration 
**************************************************************************/
void monitoring_process(void);

/**************************************************************************
* @brief	Function for load subsystem data to state packet
* @param	state_data: state packet address
**************************************************************************/
void monitoring_make_state_data(fly_protocol_state_data_t* state_data);

/**************************************************************************
* @brief	Function for get subsystem status
* @retval	Status
**************************************************************************/
uint32_t monitoring_get_status(void);

#endif /* __MONITORING_H__ */
 