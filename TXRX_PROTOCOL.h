#ifndef _TXRX_PROTOCOL_H_
#define _TXRX_PROTOCOL_H_

namespace TXRX {

	// Packet size - 38
	#pragma pack (push, 1)
	struct fly_controller_packet_t {
		uint16_t number;
		uint8_t data[32];
		uint32_t CRC;
	};
	#pragma pack (pop)
}

namespace TXRX {

	// Fly controller state data (CONTROLLER -> PC)
	#pragma pack (push, 1)
	struct state_data_t {
		uint8_t main_core_status;	// Main core status					[bitfield]
		uint8_t fly_core_mode;		// Fly core mode					[NO]
		uint8_t fly_core_status;	// Fly core status					[bitfield]

		uint8_t motors_power[4];	// Motors power						[%]
		int16_t	XYZ[3];				// X, Y, Z							[*]
		int16_t gyro_XYZ[3];		// Angular velocity					[*/sec]
		int32_t alttitude;			// H								[cm]

		uint8_t main_voltage;		// Main power supply voltage		[0.1V]
		uint8_t wireless_voltage;	// Wireless power supply voltage	[0.1V]
		uint8_t camera_voltage;		// Camera power supply voltage		[0.1V]
		uint8_t sensors_voltage;	// Sensors power supply voltage		[0.1V]
		uint8_t hull_vibration;		// Hull vibration level				[?]
		uint8_t ESC_temperature[4];	// ESC temperatures					[*C]

		//uint8_t reserved[0];
	};
	#pragma pack (pop)

	// Main core status (bitfield)
	const uint8_t MAIN_STA_NO_ERROR						= 0x00;
	const uint8_t MAIN_STA_FATAL_ERROR					= 0x01;
	const uint8_t MAIN_STA_CONFIG_ERROR					= 0x02;
	const uint8_t MAIN_STA_COMMUNICATION_BREAK			= 0x04;
	//const uint8_t MAIN_STA_COMMUNICATION_DESYNC			= 0x08;
	const uint8_t MAIN_STA_MAIN_LOW_VOLTAGE				= 0x10;
	const uint8_t MAIN_STA_WIRELESS_LOW_VOLTAGE			= 0x20;
	const uint8_t MAIN_STA_SENSORS_LOW_VOLTAGE			= 0x40;
	const uint8_t MAIN_STA_CAMERA_LOW_VOLTAGE			= 0x80;

	// Fly core mode
	const uint8_t FLY_MODE_WAIT							= 0x01;
	const uint8_t FLY_MODE_STABILIZE					= 0x02;
	const uint8_t FLY_MODE_ANGLE_PID_SETUP				= 0x03;
	const uint8_t FLY_MODE_RATE_PID_SETUP				= 0x04;
	const uint8_t FLY_MODE_DEFENCE						= 0xFF;

	// Fly core status (bitfield)
	const uint8_t FLY_STA_NO_ERROR						= 0x00;
	const uint8_t FLY_STA_FATAL_ERROR					= 0x01;
	const uint8_t FLY_STA_MPU6050_ERROR					= 0x02;
	const uint8_t FLY_STA_BMP280_ERROR					= 0x04;
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
	const uint8_t CMD_NO_COMMAND						= 0x00;
	const uint8_t CMD_SET_FLY_MODE_WAIT					= 0x01;
	const uint8_t CMD_SET_FLY_MODE_STABILIZE			= 0x02;
	const uint8_t CMD_SET_FLY_MODE_ANGLE_PID_SETUP		= 0x03;
	const uint8_t CMD_SET_FLY_MODE_RATE_PID_SETUP		= 0x04;
}

namespace TXRX {

	#pragma pack (push, 1)
	struct configuration_data_t {
		uint8_t command;
		uint8_t data[256];
		uint32_t CRC;
	};
	#pragma pack (pop)

	const uint8_t CMD_CONFIG_READ_INFORMATION			= 0x01;
	const uint8_t CMD_CONFIG_READ_MEMORY				= 0x02;
	const uint8_t CMD_CONFIG_WRITE_MEMORY				= 0x03;
	const uint8_t CMD_CONFIG_RESET						= 0xFF;
}

#endif /* _TXRX_PROTOCOL_H_ */