#ifndef __ORIENTATION_SUBSYSTEM_H__
#define __ORIENTATION_SUBSYSTEM_H__

// OSS - Orientation SubSystem

// Error codes and commands
namespace OSS {

	const uint32_t NO_ERROR				= 0x00;
	const uint32_t MPU6050_ERROR		= 0x01;
	const uint32_t BMP280_ERROR			= 0x02;

	const uint32_t CMD_DISABLE			= 0x00;
	const uint32_t CMD_ENABLE			= 0x01;
}

// External interface
namespace OSS {

	void initialize();
	void send_command(uint32_t cmd);
	void process();
	void get_position(float* XYZH);

	uint32_t get_status();
}

#endif /* __ORIENTATION_SUBSYSTEM_H__ */