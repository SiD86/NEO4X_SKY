#ifndef __MPU6050_H__
#define __MPU6050_H__

// LIBRARY VERSION: 0.0.10

const uint32_t MPU6050_DRIVER_NO_ERROR			= 0x00;
const uint32_t MPU6050_DRIVER_BUSY				= 0x01;
const uint32_t MPU6050_DRIVER_ERROR				= 0x02;

/**************************************************************************
* @brief	Function for initialize MPU6050
* @note		Write firmware for DMP
* @param	data_ready_IRQ_pin: pin for mapping data ready interrupt
* @retval	true - initialize success, false - initialize fail
**************************************************************************/
void MPU6050_initialize(uint32_t data_ready_IRQ_pin);

/**************************************************************************
* @brief	Function for DMP start
* @retval	true - success, false - error
**************************************************************************/
void MPU6050_DMP_start(void);

/**************************************************************************
* @brief	Function for DMP stop
* @retval	true - success, false - error
**************************************************************************/
void MPU6050_DMP_stop(void);

/**************************************************************************
* @brief	Function for check data ready
* @retval	true - data ready, false - data not ready or error
**************************************************************************/
bool MPU6050_is_data_ready();

/**************************************************************************
* @brief	Function for get data (XYZ) from MPU6050
* @note		This function use I2C async mode for read FIFO packet
* @param	X,Y,Z: buffers for angles
**************************************************************************/
void MPU6050_get_data(float* X, float* Y, float* Z);

/**************************************************************************
* @brief	Function for get or reset (only error) current driver status
* @retval	Current driver status
**************************************************************************/
uint32_t MPU6050_get_status();
void MPU6050_force_reset_error_status();

#endif /* __MPU6050_H__ */