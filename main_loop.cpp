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

/*
static void OSS_unit_test_loop() {

	delay(5000);

	OSS::initialize();
	Serial.print("INITIALIZE STATUS: ");
	Serial.println(OSS::get_status());

	OSS::start();
	Serial.print("RUN STATUS: ");
	Serial.println(OSS::get_status());

	while (true) {

		float XYZH[4] = { 0 };
		OSS::get_position(XYZH, true);

		static uint32_t begin = 0;

		uint32_t MPU6050_status = MPU6050_get_status();
		uint32_t BMP280_status = BMP280_get_status();
		if (MPU6050_status == MPU6050_DRIVER_ERROR || BMP280_status == BMP280_DRIVER_ERROR ||
			millis() - begin > 1000)
		{
			Serial.print("T: ");
			Serial.print(g_device_cur_process_time);
			Serial.print(" OSS_STATUS: ");
			Serial.print(OSS::get_status());
			Serial.print(" OSS_STATE: ");
			Serial.print(g_state);
			Serial.print(" OSS_ER_CNT: ");
			Serial.print(g_errors_count);
			Serial.print(" MPU_DRV: ");
			Serial.print(MPU6050_status);
			Serial.print(" BMP_DRV: ");
			Serial.print(BMP280_status);
			Serial.print(" DATA: ");
			Serial.print(XYZH[0]);
			Serial.print(" ");
			Serial.print(XYZH[1]);
			Serial.print(" ");
			Serial.print(XYZH[2]);
			Serial.print(" ");
			Serial.println(XYZH[3]);

			begin = millis();
		}
	}
}
*/

void setup() {

	Serial.begin(460800);	// DEBUG
	delay(1000);

	// Initialize I2C wire
	I2C_initialize(400000);

	//OSS_unit_test_loop();

	pinMode(2, INPUT);
	digitalWrite(2, HIGH);
	if (digitalRead(2) == LOW)
		CONFIGSS::enter_to_configuration_mode();

	if (CONFIGSS::load_and_check_configuration() == false)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_CONFIG_ERROR);

	Serial.println(g_configuration.PWM_frequency_ESC);
	

	CSS::initialize();
	ASS::initialize(0/*g_configuration.battery_low_voltage*/);

	/*while (true) {
		ASS::process();
	}*/
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
	g_sp.hardware_error_count = g_hardware_error_count;
	g_sp.software_error_count = g_software_error_count;
	g_sp.desync_count = g_desync_count;


	FLY_CORE::make_state_data(&g_sp);
	ASS::make_state_data(&g_sp);

	/*g_sp.XYZH[0] = 10;
	g_sp.XYZH[1] = 20;
	g_sp.XYZH[2] = 30;
	g_sp.XYZH[3] = 40;

	g_sp.battery_voltage = 50;

	g_sp.motors_power[0] = 70;
	g_sp.motors_power[1] = 80;
	g_sp.motors_power[2] = 90;
	g_sp.motors_power[3] = 100;*/
}
