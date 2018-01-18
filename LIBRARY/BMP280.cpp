#include <Arduino.h>
#include "I2C.h"
#include "BMP280.h"
#define MAKE_INT32(xmsb, msb, lsb, xlsb)	(((xmsb) << 24) | ((msb) << 16) | ((lsb) << 8) | ((xlsb) << 0))

#define DEVICE_ADDRESS						(0x76)
#define DEVICE_CHIP_ID						(0x58)
#define CALIB_PARAM_DATA_LEN				(24)

// Device register map
#define REG_CALIB_PARAM						(0x88)
#define REG_CHIP_ID							(0xD0)
#define REG_RESET							(0xE0)
#define REG_STATUS							(0xF3)
#define REG_CTRL_MEAS						(0xF4)
#define REG_CONFIG							(0xF5)
#define REG_MEAS_DATA						(0xF7)

// REG_CTRL_MEAS register bits
#define T_OVERSAMPLING_SKIP					(0x00 << 5)
#define T_OVERSAMPLING_X1					(0x01 << 5)
#define T_OVERSAMPLING_X2					(0x02 << 5)
#define T_OVERSAMPLING_X4					(0x03 << 5)
#define T_OVERSAMPLING_X8					(0x04 << 5)
#define T_OVERSAMPLING_X16					(0x05 << 5)

#define P_OVERSAMPLING_SKIP					(0x00 << 2)
#define P_OVERSAMPLING_X1					(0x01 << 2)
#define P_OVERSAMPLING_X2					(0x02 << 2)
#define P_OVERSAMPLING_X4					(0x03 << 2)
#define P_OVERSAMPLING_X8					(0x04 << 2)
#define P_OVERSAMPLING_X16					(0x05 << 2)

#define SLEEP_MODE							(0x00)
#define FORCE_MODE							(0x01)
#define NORMAL_MODE							(0x03)

// REG_CONFIG register bits
#define T_SB_0_5_MS							(0x00 << 5)
#define T_SB_62_5_MS						(0x01 << 5)
#define T_SB_125_MS							(0x02 << 5)
#define T_SB_250_MS							(0x03 << 5)
#define T_SB_500_MS							(0x04 << 5)
#define T_SB_1000_MS						(0x05 << 5)
#define T_SB_2000_MS						(0x06 << 5)
#define CT_SB_4000_MS						(0x07 << 5)

#define IIR_FILTER_OFF						(0x00 << 2)
#define IIR_FILTER_X2						(0x01 << 2)
#define IIR_FILTER_X4						(0x02 << 2)
#define IIR_FILTER_X8						(0x03 << 2)
#define IIR_FILTER_X16						(0x04 << 2)

#define SPI_3_WIRE_ENABLE					(0x01)
#define SPI_3_WIRE_DISABLE					(0x00)


static void calculation_PTA(uint8_t* data, uint32_t* pressure, int32_t* temperature, float* altitude);
static int32_t compensate_T(int32_t adc_T, int32_t* t_fine);
static float compensate_P(int32_t adc_P, int32_t t_fine);


struct calib_param_t {

	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
};

static calib_param_t g_calib_param;
static uint32_t g_status = BMP280_DRIVER_NO_ERROR;



//
// EXTERNAL INTERFACE
//
void BMP280_initialize() {

	g_status = BMP280_DRIVER_ERROR;

	// Read and check chip id
	uint8_t reg_data = 0;
	if (I2C_read_byte(DEVICE_ADDRESS, REG_CHIP_ID, &reg_data) == false)
		return;
	if (reg_data != DEVICE_CHIP_ID)
		return;

	// Reset device
	if (I2C_write_byte(DEVICE_ADDRESS, REG_RESET, 0xB6) == false)
		return;
	delay(50);

	// Read calibration data
	uint16_t buffer[CALIB_PARAM_DATA_LEN / 2] = {0};
	if (I2C_read_bytes(DEVICE_ADDRESS, REG_CALIB_PARAM, (uint8_t*)&buffer[0], sizeof(buffer)) == false)
		return;

	g_calib_param.dig_T1 = (uint16_t)buffer[0];
	g_calib_param.dig_T2 = (int16_t)buffer[1];
	g_calib_param.dig_T3 = (int16_t)buffer[2];

	g_calib_param.dig_P1 = (uint16_t)buffer[3];
	g_calib_param.dig_P2 = (int16_t)buffer[4];
	g_calib_param.dig_P3 = (int16_t)buffer[5];
	g_calib_param.dig_P4 = (int16_t)buffer[6];
	g_calib_param.dig_P5 = (int16_t)buffer[7];
	g_calib_param.dig_P6 = (int16_t)buffer[8];
	g_calib_param.dig_P7 = (int16_t)buffer[9];
	g_calib_param.dig_P6 = (int16_t)buffer[10];
	g_calib_param.dig_P7 = (int16_t)buffer[11];
	
	// Configure device: normal mode, P x16, T x2, IIR filter x16, T_standby = 0.5 ms
    // Noise: 0.2 Pa, 0.004 *C

	reg_data = T_SB_0_5_MS | IIR_FILTER_X16 | SPI_3_WIRE_DISABLE;
	if (I2C_write_byte(DEVICE_ADDRESS, REG_CONFIG, reg_data) == false)
		return;

	reg_data = T_OVERSAMPLING_X2 | P_OVERSAMPLING_X16 | NORMAL_MODE;
	if (I2C_write_byte(DEVICE_ADDRESS, REG_CTRL_MEAS, reg_data) == false)
		return;

	g_status = BMP280_DRIVER_NO_ERROR;
}

bool BMP280_is_data_ready() {

	g_status = BMP280_DRIVER_ERROR;

	uint8_t data = 0;
	if (I2C_read_byte(DEVICE_ADDRESS, REG_STATUS, &data) == false)
		return false;

	g_status = BMP280_DRIVER_NO_ERROR;
	return (data & 0x01) == 0;
}

void BMP280_get_data(uint32_t* pressure, int32_t* temperature, float* altitude) {

	static bool is_start_communication = false;
	if (is_start_communication == false) { // Start communication

		I2C_set_internal_address_length(1);
		if (I2C_async_read_bytes(DEVICE_ADDRESS, REG_MEAS_DATA, 6) == false) {
			g_status = BMP280_DRIVER_ERROR;
			return;
		}
		is_start_communication = true;
		g_status = BMP280_DRIVER_BUSY;
	}
	else { // Communication started. Wait complite

		// Check I2C driver status
		uint32_t status = I2C_get_status();
		if (status == I2C_DRIVER_NO_ERROR) {
			uint8_t* data = I2C_async_get_rx_buffer_address();
			calculation_PTA(data, pressure, temperature, altitude);
			is_start_communication = false;
			g_status = BMP280_DRIVER_NO_ERROR;
		}
		else if (status == I2C_DRIVER_ERROR) {
			is_start_communication = false;
			g_status = BMP280_DRIVER_ERROR;
		}
		else if (status == I2C_DRIVER_BUSY) {
			g_status = BMP280_DRIVER_BUSY;
		}
	}
}

uint32_t BMP280_get_status() {
	return g_status;
}

void BMP280_reset_status() {
	g_status = BMP280_DRIVER_NO_ERROR;
}

//
// INTERNAL INTERFACE
//
uint32_t BMP280_max_process_time = 0;
uint32_t BMP280_cur_process_time = 0;
static void calculation_PTA(uint8_t* data, uint32_t* pressure, int32_t* temperature, float* altitude) {

	uint32_t begin = micros();

	// Make values
	int32_t adc_P = MAKE_INT32(0, data[0], data[1], data[2]) >> 4;
	int32_t adc_T = MAKE_INT32(0, data[3], data[4], data[5]) >> 4;

	// Calculate pressure and temperature
	int32_t T_fine = 0;
	int32_t T = compensate_T(adc_T, &T_fine);
	float P = compensate_P(adc_P, T_fine);
	
	// Calculate altitude
	if (altitude != nullptr)
		*altitude = (-45846.2 * (pow(P / 101325.0, 0.190263) - 1.0) * 100.0);

	// Write result
	if (pressure != nullptr)
		*pressure = P;

	if (temperature != nullptr)
		*temperature = T;

	BMP280_cur_process_time = micros() - begin;
	if (BMP280_cur_process_time > BMP280_max_process_time)
		BMP280_max_process_time = BMP280_cur_process_time;
}

static int32_t compensate_T(int32_t adc_T, int32_t* t_fine) {

	int32_t dig_T1 = (int32_t)g_calib_param.dig_T1;
	int32_t dig_T2 = (int32_t)g_calib_param.dig_T2;
	int32_t dig_T3 = (int32_t)g_calib_param.dig_T3;

	int32_t var1 = (((adc_T >> 3) - (dig_T1 << 1)) * dig_T2) >> 11;
	int32_t var2 = (((((adc_T >> 4) - dig_T1) * ((adc_T >> 4) - dig_T1)) >> 12) * dig_T3) >> 14;
	int32_t fine = var1 + var2;
	int32_t T = (fine * 5 + 128) >> 8;

	*t_fine = fine;
	return T;
}

static float compensate_P(int32_t adc_P, int32_t t_fine) {

	float dig_P1 = g_calib_param.dig_P1;
	float dig_P2 = g_calib_param.dig_P2;
	float dig_P3 = g_calib_param.dig_P3;
	float dig_P4 = g_calib_param.dig_P4;
	float dig_P5 = g_calib_param.dig_P5;
	float dig_P6 = g_calib_param.dig_P6;
	float dig_P7 = g_calib_param.dig_P7;
	float dig_P8 = g_calib_param.dig_P8;
	float dig_P9 = g_calib_param.dig_P9;

	float var1 = (t_fine / 2.0) - 64000.0;
	float var2 = var1 * var1 * dig_P6 / 32768.0;
	var2 = var2 + var1 * dig_P5 * 2.0;
	var2 = var2 / 4.0 + dig_P4 * 65536.0;
	var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * dig_P1;
	if (var1 == 0)
		return 0; // avoid exception caused by division by zero

	float p = 1048576.0 - adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = dig_P9 * p * p / 2147483648.0;
	var2 = p * dig_P8 / 32768.0;
	p = p + (var1 + var2 + dig_P7) / 16.0;
	return p;
}
