#ifndef _TXRX_H_
#define _TXRX_H_

namespace TXRX {
	
	// Packet size - 32
	#pragma pack (push, 1)
	struct fly_controller_packet_t {
		uint8_t dev_addr;
		uint8_t data[30];
		uint8_t CRC;
	};
	#pragma pack (pop)
}

namespace TXRX {

	// Fly controller state data (CONTROLLER -> PC)
	#pragma pack (push, 1)
	struct state_data_t {
		uint8_t main_core_status;
		uint8_t fly_core_mode;
		uint8_t fly_core_status;

		uint8_t motors_power[4];		// Motors power [0 - 100], %
		int16_t	XYZH[4];				// X, Y, Z and alttitude
		uint16_t battery_voltage;		// Current battery voltage [0 - 65535], 0.01V

		uint8_t hardware_error_count;
		uint8_t software_error_count;
		uint8_t desync_count;
		uint8_t MPU6050_get_FIFO_size_error_count;
		uint8_t MPU6050_check_FIFO_size_error_count;
		uint8_t MPU6050_get_data_error_count;
		uint8_t I2C_nack_count;
		uint8_t I2C_timeout_count;
		
		uint8_t reserve[5];
	};
	#pragma pack (pop)

	// Main core status (bitfield)
	const uint8_t MAIN_CORE_STATUS_NO_ERROR				= 0x00;
	const uint8_t MAIN_CORE_STATUS_CONFIG_ERROR			= 0x01;
	const uint8_t MAIN_CORE_STATUS_CONN_LOST			= 0x02;
	const uint8_t MAIN_CORE_STATUS_COMM_DESYNC			= 0x04;
	const uint8_t MAIN_CORE_STATUS_LOW_VOLTAGE			= 0x08;

	// Fly core mode [0-1]
	const uint8_t FLY_CORE_STATE_FAIL					= 0x01;
	const uint8_t FLY_CORE_STATE_DISABLE				= 0x02;
	const uint8_t FLY_CORE_STATE_ENABLE					= 0x03;
	// Fly core mode [3-7]
	const uint8_t FLY_CORE_MODE_WAIT					= 0x04;
	const uint8_t FLY_CORE_MODE_STABILIZE				= 0x08;

	// Fly core status (bitfield)
	const uint8_t FLY_CORE_STATUS_NO_ERROR				= 0x00;
	const uint8_t FLY_CORE_STATUS_MPU6050_ERROR			= 0x01;
	const uint8_t FLY_CORE_STATUS_BMP280_ERROR			= 0x02;
	const uint8_t FLY_CORE_STATUS_HCSR04_ERROR			= 0x04;
	// 0x08
	// 0x10
	// 0x20
	// 0x40
	// 0x80
}

namespace TXRX {

	// Fly controller control data (PC -> CONTROLLER)
	#pragma pack (push, 1)
	struct control_data_t {
		uint8_t command;					// Fly controller command
		uint8_t arg[12];					// Fly controller command argument
		uint8_t thrust;			        	// Thrust [0; 100], %
		int16_t	XYZ[3];		            	// Destination XYZ

		uint8_t reserve[10];
	};
	#pragma pack (pop)

	// Main core commands [4 - 7] (using only system)
	const uint8_t CMD_COMMUNICATION_SILENCE		= 0x10;

	// Fly core commands [0 - 3]
	const uint8_t CMD_NO_COMMAND				= 0x00;
	const uint8_t CMD_SET_FLY_MODE_WAIT			= 0x01;
	const uint8_t CMD_SET_FLY_MODE_STABILIZE 	= 0x02;
	const uint8_t CMD_SET_XPID_PARAMS 			= 0x03;
	const uint8_t CMD_SET_YPID_PARAMS 			= 0x04;
	const uint8_t CMD_SET_ZPID_PARAMS			= 0x05;
}

namespace TXRX {

	const uint8_t UART_CMD_NO_COMMAND			= 0x00;
	const uint8_t UART_CMD_WHO_I_AM				= 0x01;
	const uint8_t UART_CMD_GET_MEMORY_BLOCK		= 0x02;
	const uint8_t UART_CMD_WRITE_CELL			= 0x03;

	const uint8_t UART_ACK_FAIL					= 0x00;
	const uint8_t UART_ACK_SUCCESS				= 0x01;
	const uint8_t UART_ACK_QUADCOPTER			= 0x02;
}

#endif // _TXRX_PROTOCOL_H_