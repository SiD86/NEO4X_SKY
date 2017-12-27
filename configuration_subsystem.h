#ifndef __CONFIGURATION_SUBSYSTEM_H__
#define __CONFIGURATION_SUBSYSTEM_H__

// CONFIGSS - Configuration SubSystem

namespace CONFIGSS {

	typedef struct {
		uint8_t memory_map_version;
		uint8_t firmware_version;
		uint32_t device_ID;

		uint8_t calibration_ESC;
		uint16_t PWM_frequency_ESC;

		uint16_t battery_low_voltage;
		uint16_t connection_timeout;

		uint16_t PID_interval;
		uint16_t PID_limit;
		uint16_t PID_start;
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

	bool load_and_check_configuration();
	void enter_to_configuration_mode();
}

extern CONFIGSS::configuration_t g_configuration;

#endif /* __CONFIGURATION_SUBSYSTEM_H__ */
