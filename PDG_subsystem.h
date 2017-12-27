#ifndef __PDG_SUBSYSTEM_H__
#define __PDG_SUBSYSTEM_H__

namespace PDGSS {

	/**************************************************************************
	* @brief	Function for initialize PWM and calibration ESC
	**************************************************************************/
	void initialize(uint32_t ESC_frequency, uint32_t is_calibration);

	/**************************************************************************
	* @brief	Function for send signal to ESC for stop motors
	**************************************************************************/
	void stop();

	/**************************************************************************
	* @brief	Function for set motor power
	* @param	power: power [0.1%] (0 - 1000)
	**************************************************************************/
	void set_power(int32_t* power);

	/**************************************************************************
	* @brief	Function for get current motor power in percent
	* @param	power: current motor power [%] (0 - 100)
	**************************************************************************/
	void get_power_in_persent(uint8_t* power);
}

#endif /* __PDG_SUBSYSTEM_H__ */