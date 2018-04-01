#ifndef _TXRX_H_
#define _TXRX_H_

namespace TXRX {

	// Packet size - 37
	#pragma pack (push, 1)
	struct fly_controller_packet_t {
		uint8_t type;
		uint8_t data[32];
		uint32_t CRC;
	};
	#pragma pack (pop)

	const uint8_t TYPE_STATE_PACKET				= 0x00;
	const uint8_t TYPE_CONTROL_PACKET			= 0x01;
	const uint8_t TYPE_CONFIG_REQ_PACKET		= 0x02;
	const uint8_t TYPE_CONFIG_ACK_PACKET		= 0x03;
}

namespace TXRX {

	// Fly controller state data (CONTROLLER -> PC)
	#pragma pack (push, 1)
	struct state_data_t {
		uint8_t main_core_status;
		uint8_t fly_core_mode;
		uint8_t fly_core_status;

		uint8_t motors_power[4];		// Motors power [0 - 100], %
		int16_t	XYZ[3];					// X, Y, Z
		int32_t alttitude;
		uint16_t battery_voltage;		// Current battery voltage [0 - 65535], 0.01V

		uint8_t hardware_error_count;
		uint8_t software_error_count;
		uint8_t desync_count;
		uint8_t MPU6050_get_FIFO_size_error_count;
		uint8_t MPU6050_check_FIFO_size_error_count;
		uint8_t MPU6050_get_data_error_count;
		uint8_t I2C_nack_count;
		uint8_t I2C_timeout_count;
		uint8_t PID_OOR_diff;
		uint8_t PID_I_OOR_diff;
		
		uint8_t reserved[3];
	};
	#pragma pack (pop)

	// Main core status (bitfield)
	const uint8_t MAIN_CORE_STATUS_NO_ERROR				= 0x00;
	const uint8_t MAIN_CORE_STATUS_CONFIG_ERROR			= 0x01;
	const uint8_t MAIN_CORE_STATUS_COMM_LOST			= 0x02;
	const uint8_t MAIN_CORE_STATUS_COMM_DESYNC			= 0x04;
	const uint8_t MAIN_CORE_STATUS_LOW_VOLTAGE			= 0x08;

	// Fly core mode
	const uint8_t FLY_CORE_MODE_WAIT					= 0x01;
	const uint8_t FLY_CORE_MODE_STABILIZE				= 0x02;
	const uint8_t FLY_CORE_MODE_PID_SETUP				= 0x03;

	// Fly core status (bitfield)
	const uint8_t FLY_CORE_STATUS_NO_ERROR				= 0x00;
	const uint8_t FLY_CORE_STATUS_FATAL_ERROR			= 0x01;
	const uint8_t FLY_CORE_STATUS_MPU6050_ERROR			= 0x02;
	const uint8_t FLY_CORE_STATUS_BMP280_ERROR			= 0x04;
}

namespace TXRX {

	// Fly controller control data (PC -> CONTROLLER)
	#pragma pack (push, 1)
	struct control_data_t {
		uint8_t command;					// Fly controller command

		uint16_t PIDX[3];					// PID for axis X
		uint16_t PIDY[3];					// PID for axis Y
		uint16_t PIDZ[3];					// PID for axis Z

		uint8_t thrust;			        	// Thrust [0; 100], %
		int16_t	XYZ[3];		            	// Destination XYZ

		uint8_t reserved[6];
	};
	#pragma pack (pop)

	// Fly core commands
	const uint8_t CMD_NO_COMMAND					= 0x00;
	const uint8_t CMD_SET_FLY_MODE_WAIT				= 0x01;
	const uint8_t CMD_SET_FLY_MODE_STABILIZE		= 0x02;
	const uint8_t CMD_SET_FLY_MODE_PID_SETUP		= 0x03;
}

namespace TXRX {

	#pragma pack (push, 1)
	struct configure_data_t {
		uint8_t cmd;
		uint8_t bytes;
		uint16_t address;
		uint8_t data[28];
	};
	#pragma pack (pop) 

	const uint8_t CFG_CMD_READ_DEVICE_ID			= 0x01;
	const uint8_t CFG_CMD_READ_BLOCK				= 0x02;
	const uint8_t CFG_CMD_WRITE_BLOCK				= 0x03;
	const uint8_t CFG_CMD_SET_DEFAULT				= 0x04;
	const uint8_t CFG_CMD_SOFTWARE_RESET			= 0x05;

	const uint8_t CFG_OPERATION_ERROR				= 0xFF;
}

#endif // _TXRX_PROTOCOL_H_