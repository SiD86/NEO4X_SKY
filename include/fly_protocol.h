#ifndef FLY_PROTOCOL_H_
#define FLY_PROTOCOL_H_

#define FLY_PROTOCOL_DATA_SIZE						(32)

// Packet size - 38
typedef struct __attribute__ ((packed)) {
	uint16_t number;
	uint8_t data[FLY_PROTOCOL_DATA_SIZE];
	uint32_t CRC;
} fly_protocol_packet_t;


//
// State data (CONTROLLER -> PC)
//
typedef struct __attribute__ ((packed)) {
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
} fly_protocol_state_data_t;

// Main core status (bitfield)
#define FP_MAIN_STA_NO_ERROR						(0x00)
#define FP_MAIN_STA_FATAL_ERROR						(0x01)
#define FP_MAIN_STA_CONFIG_ERROR					(0x02)
#define FP_MAIN_STA_COMMUNICATION_BREAK				(0x04)
//#define FP_MAIN_STA_COMMUNICATION_DESYNC			(0x08)
#define FP_MAIN_STA_MAIN_LOW_VOLTAGE				(0x10)
#define FP_MAIN_STA_WIRELESS_LOW_VOLTAGE			(0x20)
#define FP_MAIN_STA_SENSORS_LOW_VOLTAGE				(0x40)
#define FP_MAIN_STA_CAMERA_LOW_VOLTAGE				(0x80)

// Fly core mode
#define FP_FLY_MODE_WAIT							(0x01)
#define FP_FLY_MODE_STABILIZE						(0x02)
#define FP_FLY_MODE_RATE_PID_SETUP					(0x03)
#define FP_FLY_MODE_ANGLE_PID_SETUP					(0x04)
#define FP_FLY_MODE_DEFENCE							(0xFF)

// Fly core status (bitfield)
#define FP_FLY_STA_NO_ERROR							(0x00)
#define FP_FLY_STA_FATAL_ERROR						(0x01)
#define FP_FLY_STA_MPU6050_ERROR					(0x02)
#define FP_FLY_STA_BMP280_ERROR						(0x04)


//
// Control data (PC -> CONTROLLER)
//
typedef struct __attribute__ ((packed)) {
	uint8_t command;					// Fly controller command

	uint16_t PIDX[3];					// PID for axis X
	uint16_t PIDY[3];					// PID for axis Y
	uint16_t PIDZ[3];					// PID for axis Z

	uint8_t thrust;			        	// Thrust [0; 100], %
	int16_t	XYZ[3];		            	// Destination XYZ

	uint8_t reserved[6];
} fly_protocol_control_data_t;

// Fly core commands
#define FP_CMD_NO_COMMAND							(0x00)
#define FP_CMD_SET_FLY_MODE_WAIT					(0x01)
#define FP_CMD_SET_FLY_MODE_STABILIZE				(0x02)
#define FP_CMD_SET_FLY_MODE_RATE_PID_SETUP			(0x03)
#define FP_CMD_SET_FLY_MODE_ANGLE_PID_SETUP			(0x04)


//
// Wire protocol (PC <-> CONTROLLER)
//
typedef struct __attribute__ ((packed)) {
	uint8_t command;
	uint8_t data[256];
	uint32_t CRC;
} wire_packet_t;

#define WIRE_CMD_CONFIG_READ_INFORMATION			(0x01)
#define WIRE_CMD_CONFIG_READ_MEMORY					(0x02)
#define WIRE_CMD_CONFIG_WRITE_MEMORY				(0x03)
#define WIRE_CMD_CONFIG_RESET						(0xFF)

#endif /* FLY_PROTOCOL_H_ */