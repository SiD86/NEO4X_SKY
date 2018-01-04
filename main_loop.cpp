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
									 TXRX::MAIN_CORE_STATUS_CONN_LOST )
static void error_status_update();
static void make_state_packet();

static uint8_t g_status = TXRX::MAIN_CORE_STATUS_NO_ERROR;


void setup() {

	Serial.begin(460800);	// DEBUG
	delay(1000);

	// Initialize I2C wire
	I2C_initialize(400000);

	//
	// ��� ����� ������ ���������� ���� ��� ������ ��������
	//
	if (CONFIGSS::reset_configuration() == false)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_CONFIG_ERROR);

	Serial.println(g_status);
	delay(5000);


	// Check request enter to configuration mode
	pinMode(2, INPUT);
	digitalWrite(2, HIGH);
	if (digitalRead(2) == LOW)
		CONFIGSS::enter_to_configuration_mode();

	if (CONFIGSS::load_and_check_configuration() == false)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_CONFIG_ERROR);

	Serial.println(g_status);
	delay(5000);

	// Initialize subsystems and fly core
	CSS::initialize(g_configuration.send_state_data_interval, 
				    g_configuration.connection_lost_timeout);
	ASS::initialize(g_configuration.battery_low_voltage);

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


	FLY_CORE::make_state_data(&g_sp);
	ASS::make_state_data(&g_sp);
}
