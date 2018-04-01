#include <Arduino.h>
#include "LIBRARY\I2C.h"
#include "communication_subsystem.h"
#include "additional_subsystem.h"
#include "configuration_subsystem.h"
#include "fly_core.h"
#include "CONFIG.h"
#include "util.h"
#define FATAL_ERROR_MASK			(TXRX::MAIN_CORE_STATUS_CONFIG_ERROR | TXRX::MAIN_CORE_STATUS_COMM_LOST)

static void error_status_update();
static void make_state_packet();

static uint8_t g_status = TXRX::MAIN_CORE_STATUS_NO_ERROR;

void setup() {

	Serial.begin(115200);	// DEBUG

	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	delay(1000);
	digitalWrite(13, LOW);

	// Setup configuration pin
	pinMode(6, INPUT);
	digitalWrite(6, HIGH);

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

	// Initialize communication subsystem
	CSS::initialize();

	// Initialize I2C wire
	I2C_initialize(I2C_SPEED_400KHZ);

	if (digitalRead(6) == LOW) {
		Serial.println("CONFIG MODE");
		CONFIGSS::enter_to_configuration_mode();
	}
	Serial.println("MAIN MODE");
	//
	// Тут нужно ввести мониторинг пина для сброса настроек
	//
	//if (CONFIGSS::reset_configuration() == false)
	//SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_CONFIG_ERROR);

	if (CONFIGSS::load_and_check_configuration() == false)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_CONFIG_ERROR);

	// Initialize additional subsystem
	ASS::initialize();

	// Initialize fly core
	FLY_CORE::initialize();
}

void loop() {

	//
	// MAIN CORE PROCESS
	//

	// Recieve and send data
	CSS::asynchronous_process();

	// Additional subsystem process
	ASS::process();

	// Update error status
	error_status_update();



	//
	// FLY CORE PROCESS
	//

	// Make command for fly core
	uint32_t fly_core_command = FLY_CORE::INTERNAL_CMD_PROCESS;
	if (g_status & FATAL_ERROR_MASK)
		fly_core_command = FLY_CORE::INTERNAL_CMD_DISABLE;

	// Process fly core
	FLY_CORE::process(fly_core_command, &g_cp);



	//
	// CONSTRUCT STATE PACKET
	//
	
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
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_COMM_LOST);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_COMM_LOST);

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



extern uint8_t MPU6050_get_FIFO_size_error_count;
extern uint8_t MPU6050_check_FIFO_size_error_count;
extern uint8_t MPU6050_get_data_error_count;
extern volatile uint8_t I2C_nack_count;
extern volatile uint8_t I2C_timeout_count;
extern uint32_t g_PID_OOR_diff;
extern uint32_t g_PID_I_OOR_diff;
extern uint32_t g_hardware_error_count;
extern uint32_t g_software_error_count;
extern uint32_t g_desync_count;

static void make_state_packet() {

	// Clear packet
	memset(&g_sp, 0, sizeof(g_sp));

	g_sp.main_core_status = g_status;

	FLY_CORE::make_state_data(&g_sp);
	ASS::make_state_data(&g_sp);

	// Debug info
	g_sp.hardware_error_count = g_hardware_error_count;
	g_sp.software_error_count = g_software_error_count;
	g_sp.desync_count = g_desync_count;

	g_sp.MPU6050_get_FIFO_size_error_count = MPU6050_get_FIFO_size_error_count;
	g_sp.MPU6050_check_FIFO_size_error_count = MPU6050_get_FIFO_size_error_count;
	g_sp.MPU6050_get_data_error_count = MPU6050_get_data_error_count;

	g_sp.I2C_nack_count = I2C_nack_count;
	g_sp.I2C_timeout_count = I2C_timeout_count;

	g_sp.PID_OOR_diff = g_PID_OOR_diff;
	g_sp.PID_I_OOR_diff = g_PID_I_OOR_diff;
}
