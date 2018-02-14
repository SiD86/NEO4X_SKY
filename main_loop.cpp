#include <Arduino.h>
#include "LIBRARY\I2C.h"
#include "LIBRARY\MPU6050.h"
#include "LIBRARY\BMP280.h"
#include "TXRX_PROTOCOL.h"
#include "communication_subsystem.h"
#include "orientation_subsystem.h"
#include "additional_subsystem.h"
#include "fly_core.h"
#include "configuration_subsystem.h"
#include "CONFIG.h"
#include "util.h"
#define FATAL_ERROR_MASK			(TXRX::MAIN_CORE_STATUS_CONFIG_ERROR |   \
									 TXRX::MAIN_CORE_STATUS_CONN_LOST | \
									 TXRX::MAIN_CORE_STATUS_I2C_HARDWARE_ERROR)
static void error_status_update();
static void make_state_packet();

static uint8_t g_status = TXRX::MAIN_CORE_STATUS_NO_ERROR;

extern uint8_t MPU6050_get_FIFO_size_error_count;
extern uint8_t MPU6050_check_FIFO_size_error_count;
extern uint8_t MPU6050_get_data_error_count;
extern volatile uint8_t I2C_nack_count;
extern volatile uint8_t I2C_timeout_count;
extern uint32_t g_PID_OOR_count;

void setup() {

	Serial.begin(460800);	// DEBUG
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);

	delay(1000);

	pinMode(53, OUTPUT); // PB14
	pinMode(50, OUTPUT); // PC13
	pinMode(49, OUTPUT); // PC14
	pinMode(27, OUTPUT); // PD2
	pinMode(24, OUTPUT); // PA15
	pinMode(23, OUTPUT); // PA14

	CLR_DEBUG_PIN_1;
	CLR_DEBUG_PIN_2;
	CLR_DEBUG_PIN_3;
	CLR_DEBUG_PIN_4;
	CLR_DEBUG_PIN_5;
	CLR_DEBUG_PIN_6;

	// Initialize I2C wire
	I2C_initialize(I2C_SPEED_400KHZ);
	if (I2C_get_status() == I2C_DRIVER_ERROR)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_I2C_HARDWARE_ERROR);

	//
	// Тут нужно ввести мониторинг пина для сброса настроек
	//
	//if (CONFIGSS::reset_configuration() == false)
		//SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_CONFIG_ERROR);

	// Check request enter to configuration mode
	/*pinMode(2, INPUT);
	digitalWrite(2, HIGH);
	if (digitalRead(2) == LOW)
		CONFIGSS::enter_to_configuration_mode();

	if (CONFIGSS::load_and_check_configuration() == false)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_CONFIG_ERROR);*/

	g_cfg.send_state_interval = 100;		// 100 ms
	g_cfg.connection_lost_timeout = 1000;	// 1000 ms

	g_cfg.ESC_PWM_frequency = 400;			// 400 Hz
	
	g_cfg.battery_low_voltage = 1000;		// 10.00V
	
	g_cfg.PID_output_limit = 300;			// 30%
	g_cfg.PID_enable_threshold = 0;			// 0% (enable always)
	
	g_cfg.PID_X[0] = 0;
	g_cfg.PID_X[1] = 0;
	g_cfg.PID_X[2] = 0;
	g_cfg.I_X_limit = 0;
	
	g_cfg.PID_Y[0] = 0;
	g_cfg.PID_Y[1] = 0;
	g_cfg.PID_Y[2] = 0;
	g_cfg.I_Y_limit = 0;
	
	g_cfg.PID_Z[0] = 0;
	g_cfg.PID_Z[1] = 0;
	g_cfg.PID_Z[2] = 0;
	g_cfg.I_Z_limit = 0;
	
	
	// Initialize subsystems and fly core
	CSS::initialize();
	ASS::initialize();

	FLY_CORE::initialize();
}

void loop() {

	// Recieve and send data
	CSS::process();

	// Additional subsystem process
	ASS::process();

	// Update error status
	error_status_update();

	// Make command for fly core
	uint32_t fly_core_command = FLY_CORE::INTERNAL_CMD_ENABLE_CORE;
	if (g_status & FATAL_ERROR_MASK)
		fly_core_command = FLY_CORE::INTERNAL_CMD_DISABLE_CORE;

	// Process fly core
	FLY_CORE::process(fly_core_command, &g_cp);
	
	// Update state data
	make_state_packet();
}


//
// INTERNAL INTERFACE
//
static void error_status_update() {

	// Check communication subsystem status
	uint32_t status = CSS::get_status();
	if (status & CSS::CONNECTION_LOST)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_CONN_LOST);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_CONN_LOST);

	if (status & CSS::DESYNC)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_COMM_DESYNC);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_COMM_DESYNC);


	// Check additional subsystem status
	status = ASS::get_status();
	if (status & ASS::BATTERY_LOW_VOLTAGE)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_LOW_VOLTAGE);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_LOW_VOLTAGE);
}


extern uint32_t g_hardware_error_count;
extern uint32_t g_software_error_count;
extern uint32_t g_desync_count;

static void make_state_packet() {

	// Clear packet
	memset(&g_sp, 0, sizeof(g_sp));

	g_sp.main_core_status = g_status;

	// Debug info
	g_sp.hardware_error_count = g_hardware_error_count;
	g_sp.software_error_count = g_software_error_count;
	g_sp.desync_count = g_desync_count;

	g_sp.MPU6050_get_FIFO_size_error_count = MPU6050_get_FIFO_size_error_count;
	g_sp.MPU6050_check_FIFO_size_error_count = MPU6050_get_FIFO_size_error_count;
	g_sp.MPU6050_get_data_error_count = MPU6050_get_data_error_count;

	g_sp.I2C_nack_count = I2C_nack_count;
	g_sp.I2C_timeout_count = I2C_timeout_count;

	g_sp.PID_OOR_count = g_PID_OOR_count;

	FLY_CORE::make_state_data(&g_sp);
	ASS::make_state_data(&g_sp);
}
