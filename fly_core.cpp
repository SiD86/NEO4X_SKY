#include <Arduino.h>
#include "TXRX_PROTOCOL.h"
#include "fly_core.h"
#include "orientation_subsystem.h"
#include "PDG_subsystem.h"
#include "additional_subsystem.h"
#include "configuration_subsystem.h"
#include "PID_v1.h"
#include "CONFIG.h"
#include "util.h"
#define FATAL_ERROR_MASK			(TXRX::FLY_CORE_STATUS_MPU6050_ERROR)
#define SYNTHESIS(U,X,Y,Z)        	(U[0] * (X) + U[1] * (Y) + U[2] * (Z))

#define STATE_ENABLE				(0x01)
#define STATE_PROCESS				(0x02)
#define STATE_FAIL					(0x04)

static uint8_t g_state = STATE_FAIL;
static uint8_t g_fly_mode = TXRX::FLY_CORE_MODE_WAIT;
static uint8_t g_status = TXRX::FLY_CORE_STATUS_NO_ERROR;

static void user_command_handling(uint32_t cmd);
static void state_ENABLE_handling();
static void state_PROCESS_handling(TXRX::control_data_t* control_data);
static void state_FAIL_handling();

static void error_status_handling(uint32_t next_state);


//
// EXTERNAL INTERFACE
//
void FLY_CORE::initialize() {

	// Initialize subsystems
	PDGSS::initialize(g_cfg.ESC_PWM_frequency);
	PDGSS::stop();
	OSS::initialize();
	
	// Initialize PID controller channel for axis X
	PID_initialize(PID_CHANNEL_X, g_cfg.PID_output_limit, g_cfg.I_X_limit);
	PID_set_tunings(PID_CHANNEL_X, g_cfg.PID_X[0], g_cfg.PID_X[1], g_cfg.PID_X[2]);
	
	// Initialize PID controller channel for axis Y
	PID_initialize(PID_CHANNEL_Y, g_cfg.PID_output_limit, g_cfg.I_Y_limit);
	PID_set_tunings(PID_CHANNEL_Y, g_cfg.PID_Y[0], g_cfg.PID_Y[1], g_cfg.PID_Y[2]);
	
	// Initialize PID controller channel for axis Z
	PID_initialize(PID_CHANNEL_Z, g_cfg.PID_output_limit, g_cfg.I_Z_limit);
	PID_set_tunings(PID_CHANNEL_Z, g_cfg.PID_Z[0], g_cfg.PID_Z[1], g_cfg.PID_Z[2]);

	// Check errors and request go to next state
	error_status_handling(STATE_ENABLE);
}

void FLY_CORE::process(uint32_t internal_cmd, TXRX::control_data_t* control_data) {

	// Process commands
	user_command_handling(control_data->command);
	if (internal_cmd == FLY_CORE::INTERNAL_CMD_DISABLE)
		g_fly_mode = TXRX::FLY_CORE_MODE_WAIT;
	
	// Process OSS
	OSS::process();

	switch (g_state)
	{
	case STATE_ENABLE: // Call once after core initialize 
		state_ENABLE_handling();
		error_status_handling(STATE_PROCESS);
		break;

	case STATE_PROCESS:
		state_PROCESS_handling(control_data);
		error_status_handling(STATE_PROCESS);
		break;

	case STATE_FAIL:
		state_FAIL_handling();
		error_status_handling(STATE_FAIL);
		break;
	}
}

void FLY_CORE::make_state_data(TXRX::state_data_t* state_data) {

	state_data->fly_core_status = g_status;
	state_data->fly_core_mode = g_fly_mode;

	float cur_XYZH[4] = { 0 };
	OSS::get_position(cur_XYZH);
	state_data->XYZH[0] = cur_XYZH[0] * 100; // to 0.01*
	state_data->XYZH[1] = cur_XYZH[1] * 100; // to 0.01*
	state_data->XYZH[2] = cur_XYZH[2] * 100; // to 0.01*
	state_data->XYZH[3] = cur_XYZH[3] / 10; // to 0.1 meters

	PDGSS::get_power_in_persent(state_data->motors_power);
}


//
// INTERNAL INTERFACE
//
static void user_command_handling(uint32_t cmd) {

	switch (cmd)
	{
	case TXRX::CMD_NO_COMMAND:
		break;

	case TXRX::CMD_SET_FLY_MODE_WAIT:
		g_fly_mode = TXRX::FLY_CORE_MODE_WAIT;
		break;

	case TXRX::CMD_SET_FLY_MODE_STABILIZE:
		if (g_fly_mode == TXRX::FLY_CORE_MODE_STABILIZE)
			break;
		g_fly_mode = TXRX::FLY_CORE_MODE_STABILIZE;
		PID_reset(PID_CHANNEL_X);
		PID_reset(PID_CHANNEL_Y);
		PID_reset(PID_CHANNEL_Z);
		break;

	case TXRX::CMD_SET_FLY_MODE_PID_SETUP:
		if (g_fly_mode == TXRX::CMD_SET_FLY_MODE_PID_SETUP)
			break;
		g_fly_mode = TXRX::CMD_SET_FLY_MODE_PID_SETUP;
		PID_reset(PID_CHANNEL_X);
		PID_reset(PID_CHANNEL_Y);
		PID_reset(PID_CHANNEL_Z);
		break;
	}
}

static void state_ENABLE_handling() {

	// Set default fly mode
	g_fly_mode = TXRX::FLY_CORE_MODE_WAIT;

	OSS::send_command(OSS::CMD_ENABLE);
}

static void state_PROCESS_handling(TXRX::control_data_t* control_data) {

	// Get current position
	float cur_XYZH[4] = { 0 };
	bool is_position_updated = OSS::get_position(cur_XYZH);

	// ====================================================
	// Debug angle protection
	if (cur_XYZH[0] > g_cfg.angle_protect || cur_XYZH[0] < -g_cfg.angle_protect) // Axis X
		g_fly_mode = TXRX::FLY_CORE_MODE_WAIT;
	if (cur_XYZH[1] > g_cfg.angle_protect || cur_XYZH[1] < -g_cfg.angle_protect) // Axis Y
		g_fly_mode = TXRX::FLY_CORE_MODE_WAIT;
	// 
	// ====================================================
	
	// Process current fly mode
	if (g_fly_mode == TXRX::FLY_CORE_MODE_STABILIZE || g_fly_mode == TXRX::FLY_CORE_MODE_PID_SETUP) {

		// Runtime PID setup
		if (g_fly_mode == TXRX::FLY_CORE_MODE_PID_SETUP) {
			PID_set_tunings(PID_CHANNEL_X, control_data->PIDX[0] / 100.0, control_data->PIDX[1] / 100.0, control_data->PIDX[2] / 100.0);
			PID_set_tunings(PID_CHANNEL_Y, control_data->PIDY[0] / 100.0, control_data->PIDY[1] / 100.0, control_data->PIDY[2] / 100.0);
			PID_set_tunings(PID_CHANNEL_Z, control_data->PIDZ[0] / 100.0, control_data->PIDZ[1] / 100.0, control_data->PIDZ[2] / 100.0);
		}

		// Calculation PID
		float PIDU[3] = { 0 };    // X, Y, Z
		if (is_position_updated == true) {
			PIDU[0] = PID_process(PID_CHANNEL_X, cur_XYZH[0], control_data->XYZ[0]);
			PIDU[1] = PID_process(PID_CHANNEL_Y, cur_XYZH[1], control_data->XYZ[1]);
			PIDU[2] = PID_process(PID_CHANNEL_Z, cur_XYZH[2], control_data->XYZ[2]);
		}
		else {
			PIDU[0] = PID_get_last_output(PID_CHANNEL_X);
			PIDU[1] = PID_get_last_output(PID_CHANNEL_Y);
			PIDU[2] = PID_get_last_output(PID_CHANNEL_Z);
		}

		// Constrain thrust [0; 1000 - PID_output_limit]
		int32_t thrust = control_data->thrust * 10; // Scale thrust [0; 100] -> [0; 1000]
		if (thrust + g_cfg.PID_output_limit > 1000)
			thrust = 1000 - g_cfg.PID_output_limit;

		// Synthesis PIDs
		int32_t motors_power[4] = { thrust, thrust, thrust, thrust };
		if (thrust >= g_cfg.PID_enable_threshold) {
			motors_power[0] += SYNTHESIS(PIDU, -1.0F, -1.0F, -1.0F);
			motors_power[1] += SYNTHESIS(PIDU, -1.0F, +1.0F, +1.0F);
			motors_power[2] += SYNTHESIS(PIDU, +1.0F, +1.0F, -1.0F);
			motors_power[3] += SYNTHESIS(PIDU, +1.0F, -1.0F, +1.0F);
		}

		// Update ESC singal
		PDGSS::set_power(motors_power);
	}
	else if (g_fly_mode == TXRX::FLY_CORE_MODE_WAIT) {
		PDGSS::stop();
	}
}

static void state_FAIL_handling() {
	PDGSS::stop();
	OSS::send_command(OSS::CMD_DISABLE);
}


static void error_status_handling(uint32_t next_state) {

	// Check orientation subsystem error status
	uint8_t status = OSS::get_status();
	if (status & OSS::MPU6050_ERROR)
		g_status |= TXRX::FLY_CORE_STATUS_MPU6050_ERROR;
	if (status & OSS::BMP280_ERROR)
		g_status |= TXRX::FLY_CORE_STATUS_BMP280_ERROR;

	// Check fly core fatal errors
	if (g_status & FATAL_ERROR_MASK)
		g_state = STATE_FAIL;
	else
		g_state = next_state;

	// Check FAIL mode
	if (g_state == STATE_FAIL)
		g_status |= TXRX::FLY_CORE_STATUS_FATAL_ERROR;
}
