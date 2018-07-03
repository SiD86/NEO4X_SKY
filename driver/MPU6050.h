#ifndef __MPU6050_H__
#define __MPU6050_H__

#define MPU6050_DRIVER_NO_ERROR					(0x00)
#define MPU6050_DRIVER_BUSY						(0x01)
#define MPU6050_DRIVER_ERROR					(0x02)

/**************************************************************************
* @brief	Function for initialize MPU6050
* @note		Write firmware for DMP
* @retval	true - initialize success, false - initialize fail
**************************************************************************/
void MPU6050_initialize(void);

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
bool MPU6050_is_data_ready(void);

/**************************************************************************
* @brief	Function for get data (XYZ) from MPU6050
* @note		This function use I2C async mode for read FIFO packet
* @param	XY: buffer for angles
* @param	gyro_XYZ: buffer for angular velocity
**************************************************************************/
void MPU6050_get_data(float* XY, float* gyro_XYZ);

/**************************************************************************
* @brief	Functions for get and reset current driver status
* @retval	Current driver status
**************************************************************************/
uint32_t MPU6050_get_status();

#endif /* __MPU6050_H__ */