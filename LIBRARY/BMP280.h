#ifndef __BMP280_H__
#define __BMP280_H__

// LIBRARY VERSION: 0.0.8

const uint32_t BMP280_DRIVER_NO_ERROR		= 0x00;
const uint32_t BMP280_DRIVER_BUSY			= 0x01;
const uint32_t BMP280_DRIVER_ERROR			= 0x02;

/**************************************************************************
* @brief	Function for initialize BMP280
* @note		Function read calibration data from sensor and check chip id
* @retval	true - success, false - I2C error or invalid chip id
**************************************************************************/
void BMP280_initialize();

/**************************************************************************
* @brief	Function for check data ready
* @param	pressure: pointer to buffer for pressure
* @param	temperature: pointer to buffer for temperature
* @retval	true - data is ready, false - I2C error or data not ready
**************************************************************************/
bool BMP280_is_data_ready();

/**************************************************************************
* @brief	Function for calculation pressure and temperature
* @param	pressure: pointer to buffer for pressure (Pa)
* @param	temperature: pointer to buffer for temperature (0.01 *C)
* @param	altitude: pointer to buffer for altitude (cm)
* @retval	true - success, false - I2C error or calculation error
**************************************************************************/
void BMP280_get_data(uint32_t* pressure, int32_t* temperature, float* altitude);

/**************************************************************************
* @brief	Functions for get and reset current driver status
* @retval	Current driver status
**************************************************************************/
uint32_t BMP280_get_status();

#endif /* __BMP280_H__ */