#ifndef PDG_H_
#define PDG_H_

/**************************************************************************
* @brief	Function for initialize PWM and calibration ESC
**************************************************************************/
void pdg_initialize(uint32_t ESC_frequency);

/**************************************************************************
* @brief	Function for send signal to ESC for stop motors
**************************************************************************/
void pdg_stop(void);

/**************************************************************************
* @brief	Function for set motor power
* @param	power: power [0.1%] (0 - 1000)
**************************************************************************/
void pdg_set_power(int32_t* power);

/**************************************************************************
* @brief	Function for get current motor power in percent
* @param	power: current motor power [%] (0 - 100)
**************************************************************************/
void pdg_get_power_in_persent(uint8_t* power);

#endif /* PDG_SUBSYSTEM_H_ */