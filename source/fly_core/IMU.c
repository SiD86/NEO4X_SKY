#include <stdbool.h>
#include "sam3x8e.h"
#include "systimer.h"
#include "MPU6050.h"
#include "BMP280.h"
#include "imu.h"
#include "utils.h"
#include "UART.h"
#define MAX_ERROR_COUNT					(10)

#define STATE_DISABLE					(0x00)
#define STATE_MPU6050_CHECK_RDY			(0x01)
#define STATE_MPU6050_GET_DATA			(0x02)
#define STATE_BMP280_CHECK_RDY			(0x03)
#define STATE_BMP280_GET_DATA			(0x04)

static void error_status_update(bool check_MPU6050, bool check_BMP280, bool is_fatal_operation);
static void state_MPU6050_CHECK_RDY_handler();
static void state_MPU6050_GET_DATA_handler();
static void state_BMP280_CHECK_RDY_handler();
static void state_BMP280_GET_DATA_handler();

static uint32_t g_state = STATE_DISABLE;
static uint32_t g_status = IMU_NO_ERROR;
static uint32_t g_MPU6050_error_count = 0;
static uint32_t g_BMP280_error_count = 0;
static float g_XYZH[4] = { 0 };
static float g_gyro_XYZ[4] = { 0 };
static bool g_is_position_updated = false;


//
// EXTERNAL INTERFACE
//
void imu_initialize() {

	// Initialize MPU6050
	MPU6050_initialize();
	
	// Initialize BMP280
	BMP280_initialize();

	g_state = STATE_DISABLE;
	error_status_update(true, true, true);
}

void imu_enable() {

	if (IS_BIT_CLEAR(g_status, IMU_MPU6050_ERROR))
		MPU6050_DMP_start();
	g_state = STATE_MPU6050_CHECK_RDY;
	g_is_position_updated = false;

	g_MPU6050_error_count = 0;
	g_BMP280_error_count = 0;
	error_status_update(true, false, true);
}

void imu_disable() {

	if (IS_BIT_CLEAR(g_status, IMU_MPU6050_ERROR))
		MPU6050_DMP_stop();
	g_state = STATE_DISABLE;
	g_is_position_updated = false;

	g_MPU6050_error_count = 0;
	g_BMP280_error_count = 0;
	error_status_update(true, false, true);
}

void imu_process() {

	switch (g_state)
	{
	case STATE_DISABLE:
		break;

	case STATE_MPU6050_CHECK_RDY:	// Sync operation
		state_MPU6050_CHECK_RDY_handler();
		break;

	case STATE_MPU6050_GET_DATA:	// Async operation
		state_MPU6050_GET_DATA_handler();
		break;

	case STATE_BMP280_CHECK_RDY:	// Sync operation
		state_BMP280_CHECK_RDY_handler();
		break;

	case STATE_BMP280_GET_DATA:		// Async operation
		state_BMP280_GET_DATA_handler();
		break;
	}
}

void imu_get_position(float* XYZH, float* gyro_XYZ) {

	XYZH[0] = g_XYZH[0];
	XYZH[1] = g_XYZH[1];
	XYZH[2] = g_XYZH[2];
	XYZH[3] = g_XYZH[3];

	gyro_XYZ[0] = g_gyro_XYZ[0];
	gyro_XYZ[1] = g_gyro_XYZ[1];
	gyro_XYZ[2] = g_gyro_XYZ[2];
}

bool imu_is_position_updated() {
	bool temp = g_is_position_updated;
	g_is_position_updated = false;
	return temp;
}

uint32_t imu_get_status() {
	return g_status;
}



//
// INTERNAL INTERFACE
//
/**************************************************************************
* @brief	MPU6050 check data ready state handler
* @note 	Go to STATE_MPU6050_GET_DATA (data ready) 
* @note 	Go to STATE_MPU6050_CHECK_RDY (error or data not ready)
**************************************************************************/
static void state_MPU6050_CHECK_RDY_handler() {

	// Check device status
	if (IS_BIT_SET(g_status, IMU_MPU6050_ERROR)) {
		g_state = STATE_BMP280_CHECK_RDY;
	}

	// Check data ready
	if (MPU6050_is_data_ready() == true) {
		g_state = STATE_MPU6050_GET_DATA;
	}

	error_status_update(true, false, false);
}

/**************************************************************************
* @brief	MPU6050 read data state handler
* @note 	Go to STATE_BMP280_CHECK_RDY (always)
**************************************************************************/
static void state_MPU6050_GET_DATA_handler() {

	// Check device status
	if (IS_BIT_SET(g_status, IMU_MPU6050_ERROR)) {
		g_state = STATE_BMP280_CHECK_RDY;
	}

	// Process state
	MPU6050_get_data(g_XYZH, g_gyro_XYZ);
	if (MPU6050_get_status() != MPU6050_DRIVER_BUSY) {
		g_state = STATE_BMP280_CHECK_RDY;
		g_is_position_updated = true;
	}

	error_status_update(true, false, false);
}

/**************************************************************************
* @brief	BMP280 check data ready state handler
* @note 	Go to STATE_BMP280_GET_DATA (data ready)
* @note 	Go to STATE_MPU6050_CHECK_RDY (error or data not ready)
**************************************************************************/
static void state_BMP280_CHECK_RDY_handler() {

	// Check device status
	if (IS_BIT_SET(g_status, IMU_BMP280_ERROR)) {
		g_state = STATE_MPU6050_CHECK_RDY;
	}

	// Check measurement period
	static uint32_t prev_check_time = 0;
	if (millis() - prev_check_time < 25) {
		g_state = STATE_MPU6050_CHECK_RDY;
		return;
	}
	prev_check_time = millis();

	if (BMP280_is_data_ready() == true)
		g_state = STATE_BMP280_GET_DATA;
	else
		g_state = STATE_MPU6050_CHECK_RDY;

	error_status_update(false, true, false);
}

/**************************************************************************
* @brief	BMP280 read data state handler
* @note 	Go to STATE_MPU6050_CHECK_RDY (always)
**************************************************************************/
static void state_BMP280_GET_DATA_handler() {

	// Check device status
	if (IS_BIT_SET(g_status, IMU_BMP280_ERROR)) {
		g_state = STATE_MPU6050_CHECK_RDY;
	}

	// Get data
	BMP280_get_data(&g_XYZH[3]);
	if (BMP280_get_status() != BMP280_DRIVER_BUSY) {
		g_state = STATE_MPU6050_CHECK_RDY;
	}

	error_status_update(false, true, false);
}

/**************************************************************************
* @brief	Function for check status devices and update subsystem status
* @param	is_fatal_operation: true - error bit set skip error counter 
**************************************************************************/
static void error_status_update(bool check_MPU6050, bool check_BMP280, bool is_fatal_operation) {

	// Check MPU6050 status
	if (check_MPU6050 == true && IS_BIT_CLEAR(g_status, IMU_MPU6050_ERROR)) {

		uint32_t status = MPU6050_get_status();
		if (status == MPU6050_DRIVER_ERROR) {
		
			++g_MPU6050_error_count;
			if (g_MPU6050_error_count >= MAX_ERROR_COUNT || is_fatal_operation == true)
				SET_BIT(g_status, IMU_MPU6050_ERROR);
		}
		else if (status == MPU6050_DRIVER_NO_ERROR) {
			g_MPU6050_error_count = 0;
		}
	}

	// Check BMP280 status
	if (check_BMP280 == true && IS_BIT_CLEAR(g_status, IMU_BMP280_ERROR)) {

		uint32_t status = BMP280_get_status();
		if (status == BMP280_DRIVER_ERROR) {

			++g_BMP280_error_count;
			if (g_BMP280_error_count >= MAX_ERROR_COUNT || is_fatal_operation == true)
				SET_BIT(g_status, IMU_BMP280_ERROR);
		}
		else if (status == BMP280_DRIVER_NO_ERROR) {
			g_BMP280_error_count = 0;
		}
	}
}
