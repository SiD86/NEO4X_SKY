#include <Arduino.h>
#include "library\MPU6050.h"
#include "LIBRARY\BMP280.h"
#include "orientation_subsystem.h"
#include "CONFIG.h"
#include "util.h"
#define MAX_ERROR_COUNT									(10)

static void error_status_update(bool check_MPU6050, bool check_BMP280, bool is_fatal_operation);
static void state_MPU6050_CHECK_RDY_handler();
static void state_MPU6050_GET_DATA_handler();
static void state_BMP280_CHECK_RDY_handler();
static void state_BMP280_GET_DATA_handler();

static const uint32_t STATE_DISABLE						= 0x00;
static const uint32_t STATE_MPU6050_CHECK_RDY			= 0x01;
static const uint32_t STATE_MPU6050_GET_DATA			= 0x02;
static const uint32_t STATE_BMP280_CHECK_RDY			= 0x03;
static const uint32_t STATE_BMP280_GET_DATA				= 0x04;

static uint32_t g_state = STATE_DISABLE;
static uint32_t g_status = OSS::NO_ERROR;
static uint32_t g_MPU6050_error_count = 0;
static uint32_t g_BMP280_error_count = 0;
static float g_XYZH[4] = { 0 };


//
// EXTERNAL INTERFACE
//
void OSS::initialize() {

	// Initialize MPU6050
	MPU6050_initialize(OSS_MPU6050_DATA_RDY_PIN);
	
	// Initialize BMP280
	BMP280_initialize();

	// Initialize HCSR04
	//

	g_state = STATE_DISABLE;
	error_status_update(true, true, true);
}

void OSS::send_command(uint32_t cmd) {

	switch (cmd) 
	{
	case OSS::CMD_DISABLE:
		MPU6050_DMP_stop();
		g_state = STATE_DISABLE;
		break;

	case OSS::CMD_ENABLE:
		MPU6050_DMP_start();
		g_state = STATE_MPU6050_CHECK_RDY;
		break;

	case OSS::CMD_FORCE_SHUTDOWN:
		g_state = STATE_DISABLE;
		break;
	}

	g_MPU6050_error_count = 0;
	g_BMP280_error_count = 0;
	error_status_update(true, false, true);
}

void OSS::process() {

	switch (g_state)
	{
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

	case STATE_DISABLE:
		break;
	}
}

void OSS::get_position(float* XYZH) {
	memcpy(XYZH, g_XYZH, sizeof(g_XYZH));
}

uint32_t OSS::get_status() {
	return g_status;
}



//
// INTERNAL INTERFACE
//
/**************************************************************************
* @brief	MPU6050 check data ready state handler
* @note 	Go to STATE_MPU6050_GET_DATA (data ready) 
* @note 	Go to STATE_BMP280_CHECK_RDY (error or data not ready)
**************************************************************************/
static void state_MPU6050_CHECK_RDY_handler() {

	// Check device status
	if (g_status & OSS::MPU6050_ERROR)
		g_state = STATE_BMP280_CHECK_RDY;

	// Process state
	if (MPU6050_is_data_ready() == true)
		g_state = STATE_MPU6050_GET_DATA;
	else
		g_state = STATE_BMP280_CHECK_RDY;

	error_status_update(true, false, false);
}

/**************************************************************************
* @brief	MPU6050 read data state handler
* @note 	Go to STATE_BMP280_CHECK_RDY (always)
**************************************************************************/
static void state_MPU6050_GET_DATA_handler() {

	// Check device status
	if (g_status & OSS::MPU6050_ERROR)
		g_state = STATE_BMP280_CHECK_RDY;

	// Process state
	MPU6050_get_data(&g_XYZH[0], &g_XYZH[1], &g_XYZH[2]);
	if (MPU6050_get_status() != MPU6050_DRIVER_BUSY)
		g_state = STATE_BMP280_CHECK_RDY;

	error_status_update(true, false, false);
}

/**************************************************************************
* @brief	BMP280 check data ready state handler
* @note 	Go to STATE_BMP280_GET_DATA (data ready)
* @note 	Go to STATE_MPU6050_CHECK_RDY (error or data not ready)
**************************************************************************/
static void state_BMP280_CHECK_RDY_handler() {

	// Check device status
	if (g_status & OSS::BMP280_ERROR)
		g_state = STATE_MPU6050_CHECK_RDY;

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
	if (g_status & OSS::BMP280_ERROR)
		g_state = STATE_MPU6050_CHECK_RDY;

	BMP280_get_data(nullptr, nullptr, &g_XYZH[3]);
	if (BMP280_get_status() != BMP280_DRIVER_BUSY)
		g_state = STATE_MPU6050_CHECK_RDY;

	error_status_update(false, true, false);
}

/**************************************************************************
* @brief	Function for check status devices and update subsystem status
* @param	is_fatal_operation: true - error bit set skip error counter 
**************************************************************************/
static void error_status_update(bool check_MPU6050, bool check_BMP280, bool is_fatal_operation) {

	// Check MPU6050 status
	if (check_MPU6050 == true) {

		if (MPU6050_get_status() == MPU6050_DRIVER_ERROR) {
			if (++g_MPU6050_error_count >= MAX_ERROR_COUNT || is_fatal_operation == true)
				SET_STATUS_BIT(g_status, OSS::MPU6050_ERROR);
		}
		MPU6050_reset_status();
	}

	// Check BMP280 status
	if (check_BMP280 == true) {

		if (BMP280_get_status() == BMP280_DRIVER_ERROR) {
			if (++g_BMP280_error_count >= MAX_ERROR_COUNT || is_fatal_operation == true)
				SET_STATUS_BIT(g_status, OSS::BMP280_ERROR);
		}
		BMP280_reset_status();
	}

	// Reset error counters
	static uint32_t prev_check_time = 0;
	if (millis() - prev_check_time > 1000) {

		static uint32_t prev_MPU6050_error_count = 0;
		if (prev_MPU6050_error_count == g_MPU6050_error_count)
			g_MPU6050_error_count = 0;
		
		static uint32_t prev_BMP280_error_count = 0;
		if (prev_BMP280_error_count == g_BMP280_error_count)
			g_BMP280_error_count = 0;

		prev_check_time = millis();
		prev_MPU6050_error_count = g_MPU6050_error_count;
		prev_BMP280_error_count = g_BMP280_error_count;
	}
}


