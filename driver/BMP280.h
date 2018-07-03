#ifndef _BMP280_H
#define _BMP280_H

#define BMP280_DRIVER_NO_ERROR				(0x00)
#define BMP280_DRIVER_BUSY					(0x01)
#define BMP280_DRIVER_ERROR					(0x02)

/**************************************************************************
* @brief	Function for initialize BMP280
* @note		Function read calibration data from sensor and check chip id
* @retval	true - success, false - I2C error or invalid chip id
**************************************************************************/
void BMP280_initialize(void);

/**************************************************************************
* @brief	Function for check data ready
* @param	pressure: pointer to buffer for pressure
* @param	temperature: pointer to buffer for temperature
* @retval	true - data is ready, false - I2C error or data not ready
**************************************************************************/
bool BMP280_is_data_ready(void);

/**************************************************************************
* @brief	Function for calculation pressure and temperature
* @param	altitude: pointer to buffer for altitude (cm)
* @retval	true - success, false - I2C error or calculation error
**************************************************************************/
void BMP280_get_data(float* altitude);

/**************************************************************************
* @brief	Functions for get and reset current driver status
* @retval	Current driver status
**************************************************************************/
uint32_t BMP280_get_status(void);

#endif /* BMP280_H_ */