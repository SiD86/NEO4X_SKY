#include <stdbool.h>
#include "sam3x8e.h"
#include "fly_core.h"
#include "IMU.h"
#include "PDG.h"
#include "configuration.h"
#include "pid_controller.h"
#include "utils.h"
#define FATAL_ERRORS_MASK			(FP_FLY_STA_MPU6050_ERROR)
#define SYNTHESIS(U,X,Y,Z)        	(U[0] * (X) + U[1] * (Y) + U[2] * (Z))

#define STATE_ENABLE				(0x01)
#define STATE_PROCESS				(0x02)
#define STATE_FAIL					(0x03)

static void user_command_handling(uint32_t cmd);
static void state_ENABLE_handling();
static void state_PROCESS_handling(fly_protocol_control_data_t* control_data);
static void state_FAIL_handling();
static void defence_process(float* XYZH);
static void error_status_update();
static void request_state(uint32_t next_state);

static uint8_t g_state	  = STATE_FAIL;
static uint8_t g_fly_mode = FP_FLY_MODE_WAIT;
static uint8_t g_status	  = FP_FLY_STA_NO_ERROR;


//
// EXTERNAL INTERFACE
//
void fly_core_initialize() {

	// Initialize subsystems
	pdg_initialize(g_cfg.ESC_PWM_frequency);
	imu_initialize();
	
	//
	// Initialize rate PID
	//
	pid_initialize(PID_CHANNEL_RATE_X, g_cfg.pid_output_limit, g_cfg.rate_i_x_limit);
	pid_set_tunings(PID_CHANNEL_RATE_X, g_cfg.rate_pid_x[0], g_cfg.rate_pid_x[1], g_cfg.rate_pid_x[2]);

	pid_initialize(PID_CHANNEL_RATE_Y, g_cfg.pid_output_limit, g_cfg.rate_i_y_limit);
	pid_set_tunings(PID_CHANNEL_RATE_Y, g_cfg.rate_pid_y[0], g_cfg.rate_pid_y[1], g_cfg.rate_pid_y[2]);

	pid_initialize(PID_CHANNEL_RATE_Z, g_cfg.pid_output_limit, g_cfg.rate_i_z_limit);
	pid_set_tunings(PID_CHANNEL_RATE_Z, g_cfg.rate_pid_z[0], g_cfg.rate_pid_z[1], g_cfg.rate_pid_z[2]);

	//
	// Initialize angle PID
	//
	pid_initialize(PID_CHANNEL_ANGLE_X, g_cfg.pid_output_limit, g_cfg.angle_i_x_limit);
	pid_set_tunings(PID_CHANNEL_ANGLE_X, g_cfg.angle_pid_x[0], g_cfg.angle_pid_x[1], g_cfg.angle_pid_x[2]);

	pid_initialize(PID_CHANNEL_ANGLE_Y, g_cfg.pid_output_limit, g_cfg.angle_i_y_limit);
	pid_set_tunings(PID_CHANNEL_ANGLE_Y, g_cfg.angle_pid_y[0], g_cfg.angle_pid_y[1], g_cfg.angle_pid_y[2]);

	pid_initialize(PID_CHANNEL_ANGLE_Z, g_cfg.pid_output_limit, g_cfg.angle_i_z_limit);
	pid_set_tunings(PID_CHANNEL_ANGLE_Z, g_cfg.angle_pid_z[0], g_cfg.angle_pid_z[1], g_cfg.angle_pid_z[2]);


	// Check errors and request go to next state
	error_status_update();
	request_state(STATE_ENABLE);
}

void fly_core_process(fly_protocol_control_data_t* control_data) {

	user_command_handling(control_data->command);

	// Process OSS
	imu_process();

	switch (g_state)
	{
	case STATE_ENABLE: // Call once after core initialize 
		state_ENABLE_handling();
		error_status_update();
		request_state(STATE_PROCESS);
		break;

	case STATE_PROCESS:
		state_PROCESS_handling(control_data);
		error_status_update();
		request_state(STATE_PROCESS);
		break;

	case STATE_FAIL:
		state_FAIL_handling();
		error_status_update();
		request_state(STATE_FAIL);
		break;
	}
}

void fly_core_make_state_data(fly_protocol_state_data_t* state_data) {

	state_data->fly_core_status = g_status;
	state_data->fly_core_mode = g_fly_mode;

	float XYZH[4] = { 0 };
	float gyro_XYZ[3] = { 0 };
	imu_get_position(XYZH, gyro_XYZ);

	state_data->XYZ[0] = XYZH[0] * 100; // to 0.01*
	state_data->XYZ[1] = XYZH[1] * 100; // to 0.01*
	state_data->XYZ[2] = XYZH[2] * 100; // to 0.01*

	state_data->gyro_XYZ[0] = gyro_XYZ[0] * 100; // to 0.01*
	state_data->gyro_XYZ[1] = gyro_XYZ[1] * 100; // to 0.01*
	state_data->gyro_XYZ[2] = gyro_XYZ[2] * 100; // to 0.01*

	state_data->alttitude = XYZH[3];

	pdg_get_power_in_persent(state_data->motors_power);
}


//
// INTERNAL INTERFACE
//
static void user_command_handling(uint32_t cmd) {

	switch (cmd)
	{
	case FP_CMD_NO_COMMAND:
		break;

	case FP_CMD_SET_FLY_MODE_WAIT:
		g_fly_mode = FP_FLY_MODE_WAIT;
		break;

	case FP_CMD_SET_FLY_MODE_STABILIZE:
		if (g_fly_mode == FP_FLY_MODE_STABILIZE) {
			break;
		}
		g_fly_mode = FP_FLY_MODE_STABILIZE;
		pid_reset_all_channels();
		break;
		
	case FP_CMD_SET_FLY_MODE_ANGLE_PID_SETUP:
		if (g_fly_mode == FP_CMD_SET_FLY_MODE_ANGLE_PID_SETUP) {
			break;
		}
		g_fly_mode = FP_FLY_MODE_ANGLE_PID_SETUP;
		pid_reset_all_channels();
		break;

	case FP_CMD_SET_FLY_MODE_RATE_PID_SETUP:
		if (g_fly_mode == FP_CMD_SET_FLY_MODE_RATE_PID_SETUP) {
			break;
		}
		g_fly_mode = FP_FLY_MODE_RATE_PID_SETUP;
		pid_reset_all_channels();
		break;
	}
}

static void state_ENABLE_handling() {

	// Set default fly mode
	g_fly_mode = FP_FLY_MODE_WAIT;

	imu_enable();
}

static void state_PROCESS_handling(fly_protocol_control_data_t* control_data) {

	// Get current position
	float XYZH[4] = { 0 };
	float gyro_XYZ[3] = { 0 };
	imu_get_position(XYZH, gyro_XYZ);

	// Defense process
	defence_process(XYZH);

	// Current position is updated?
	bool is_position_updated = imu_is_position_updated();
	
	// Process current fly mode
	if (g_fly_mode == FP_FLY_MODE_STABILIZE || g_fly_mode == FP_FLY_MODE_RATE_PID_SETUP || g_fly_mode == FP_FLY_MODE_ANGLE_PID_SETUP) {
		
		// PID process only after IMU update
		if (is_position_updated == false) {
			return;
		}

		// Runtime PID setup
		if (g_fly_mode == FP_FLY_MODE_RATE_PID_SETUP) {
			pid_set_tunings(PID_CHANNEL_RATE_X, control_data->PIDX[0] / 100.0, control_data->PIDX[1] / 100.0, control_data->PIDX[2] / 100.0);
			pid_set_tunings(PID_CHANNEL_RATE_Y, control_data->PIDY[0] / 100.0, control_data->PIDY[1] / 100.0, control_data->PIDY[2] / 100.0);
			pid_set_tunings(PID_CHANNEL_RATE_Z, control_data->PIDZ[0] / 100.0, control_data->PIDZ[1] / 100.0, control_data->PIDZ[2] / 100.0);
		}
		if (g_fly_mode == FP_FLY_MODE_ANGLE_PID_SETUP) {
			pid_set_tunings(PID_CHANNEL_ANGLE_X, control_data->PIDX[0] / 100.0, control_data->PIDX[1] / 100.0, control_data->PIDX[2] / 100.0);
			pid_set_tunings(PID_CHANNEL_ANGLE_Y, control_data->PIDY[0] / 100.0, control_data->PIDY[1] / 100.0, control_data->PIDY[2] / 100.0);
			pid_set_tunings(PID_CHANNEL_ANGLE_Z, control_data->PIDZ[0] / 100.0, control_data->PIDZ[1] / 100.0, control_data->PIDZ[2] / 100.0);
		}

		// Calculation rate PID
		float PIDU[3] = { 0 };    // X, Y, Z
		if (g_fly_mode == FP_FLY_MODE_RATE_PID_SETUP) {
			
			// Calculate internal PID loop (input: degree in second, output: motor power)
			PIDU[0] = pid_calculate(PID_CHANNEL_RATE_X, gyro_XYZ[0], control_data->XYZ[0]);
			PIDU[1] = pid_calculate(PID_CHANNEL_RATE_Y, gyro_XYZ[1], control_data->XYZ[1]);
			PIDU[2] = pid_calculate(PID_CHANNEL_RATE_Z, gyro_XYZ[2], control_data->XYZ[2]);
		}
		else { // FP_FLY_MODE_ANGLE_PID_SETUP or FP_FLY_MODE_STABILIZE fly mode
			
			// Calculate external PID loop (input: angle, output: degree in second)
			float internal_out[3] = {0};
			internal_out[0] = pid_calculate(PID_CHANNEL_ANGLE_X, XYZH[0], control_data->XYZ[0]);
			internal_out[1] = pid_calculate(PID_CHANNEL_ANGLE_Y, XYZH[1], control_data->XYZ[1]);
			internal_out[2] = pid_calculate(PID_CHANNEL_ANGLE_Z, XYZH[2], control_data->XYZ[2]);
			
			// Calculate internal PID loop (input: degree in second, output: motor power)
			PIDU[0] = pid_calculate(PID_CHANNEL_RATE_X, gyro_XYZ[0], internal_out[0]);
			PIDU[1] = pid_calculate(PID_CHANNEL_RATE_Y, gyro_XYZ[1], internal_out[1]);
			PIDU[2] = pid_calculate(PID_CHANNEL_RATE_Z, gyro_XYZ[2], internal_out[2]);
		}

		// Scale thrust [0; 100] -> [0; 1000]
		int32_t thrust = control_data->thrust * 10; 
		
		// Constrain thrust [0; 1000 - PID_output_limit]
		if (thrust > 1000 - g_cfg.pid_output_limit) {
			thrust = 1000 - g_cfg.pid_output_limit;
		}

		// Calculate motor power
		int32_t motors_power[4] = { thrust, thrust, thrust, thrust };
		if (thrust >= g_cfg.pid_enable_threshold) {
			motors_power[0] += SYNTHESIS(PIDU, -1.0F, -1.0F, -1.0F);
			motors_power[1] += SYNTHESIS(PIDU, -1.0F, +1.0F, +1.0F);
			motors_power[2] += SYNTHESIS(PIDU, +1.0F, +1.0F, -1.0F);
			motors_power[3] += SYNTHESIS(PIDU, +1.0F, -1.0F, +1.0F);
		}

		// Set motor power
		pdg_set_power(motors_power);
	}
	else if (g_fly_mode == FP_FLY_MODE_WAIT) {
		pdg_stop();
	}
	else if (g_fly_mode == FP_FLY_MODE_DEFENCE) {
		pdg_stop();
	}
}

static void state_FAIL_handling() {
	pdg_stop();
}

static void defence_process(float* XYZH) {

	// ====================================================
	// Debug angle protection

	// Check angle on axis X
	if (XYZH[0] > g_cfg.angle_protect || XYZH[0] < -g_cfg.angle_protect)
		g_fly_mode = FP_FLY_MODE_DEFENCE;

	// Check angle on axis Y
	if (XYZH[1] > g_cfg.angle_protect || XYZH[1] < -g_cfg.angle_protect)
		g_fly_mode = FP_FLY_MODE_DEFENCE;

	//
	// ====================================================
}


static void error_status_update() {

	// Check orientation subsystem error status
	uint32_t status = imu_get_status();
	if (IS_BIT_SET(status, IMU_MPU6050_ERROR) == true) {
		SET_BIT(g_status, FP_FLY_STA_MPU6050_ERROR);
	}
	if (IS_BIT_SET(status, IMU_BMP280_ERROR) == true) {
		SET_BIT(g_status, FP_FLY_STA_BMP280_ERROR);
	}

	// Check FAIL mode
	if (g_status & FATAL_ERRORS_MASK) {
		SET_BIT(g_status, FP_FLY_STA_FATAL_ERROR);
	}
}

// Call after error_status_update()
static void request_state(uint32_t next_state) {

	// Check fly core fatal errors
	if (IS_BIT_SET(g_status, FP_FLY_STA_FATAL_ERROR) == true) {
		g_state = STATE_FAIL;
	}
	else {
		g_state = next_state;
	}
}
