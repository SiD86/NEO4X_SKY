#ifndef __CONFIGURATION_SUBSYSTEM_H__
#define __CONFIGURATION_SUBSYSTEM_H__

// CONFIGSS - Configuration SubSystem

namespace CONFIGSS {

	typedef struct {

		uint16_t memory_map_version;
		uint16_t FW_version;
		uint16_t device_ID;

		uint16_t connection_lost_timeout;
		uint16_t desync_silence_window_time;
		uint16_t send_state_interval;

		uint8_t angle_protect;
		uint16_t ESC_PWM_frequency;

		uint16_t battery_low_voltage;

		uint16_t PID_output_limit;
		uint16_t PID_enable_threshold;
		float PID_X[3];
		float I_X_limit;
		float PID_Y[3];
		float I_Y_limit;
		float PID_Z[3];
		float I_Z_limit;
		float PID_H[3];
		float I_H_limit;

	} configuration_t;
}

// External interface
namespace CONFIGSS {
	bool intialize();
}

extern CONFIGSS::configuration_t g_cfg;

#endif /* __CONFIGURATION_SUBSYSTEM_H__ */
