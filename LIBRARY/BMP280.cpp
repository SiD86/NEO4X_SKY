#include <Arduino.h>
#include "I2C.h"
#include "BMP280.h"
#define I2C_ADDRESS							0x76
////////////////////////////////////////////////////////
// Register "chip_id"
#define WHO_I_AM_REG						0xD0
#define CHIP_ID								0x58
//
////////////////////////////////////////////////////////
// Registers "calibration"
#define CALIBRATION_REG						0x88
#define CALIBRATION_DATA_SIZE				24
//
////////////////////////////////////////////////////////
// Register "status"
#define STATUS_REG							0xF3
//
////////////////////////////////////////////////////////
// Register "reset"
#define CTRL_RESET_REG						0xE0
#define CTRL_RESET_RESET					0xB6
//
////////////////////////////////////////////////////////
// Register "ctrl_meas"
#define CTRL_MEAS_REG						0xF4
// osrs_t
#define CTRL_MEAS_OSRS_T_SKIP				0x00
#define CTRL_MEAS_OSRS_T_X1					0x20
#define CTRL_MEAS_OSRS_T_X2					0x40
#define CTRL_MEAS_OSRS_T_X4					0x60
#define CTRL_MEAS_OSRS_T_X8					0x80
#define CTRL_MEAS_OSRS_T_X16				0xE0
// osrs_p
#define CTRL_MEAS_OSRS_P_SKIP				0x00
#define CTRL_MEAS_OSRS_P_X1					0x04
#define CTRL_MEAS_OSRS_P_X2					0x08
#define CTRL_MEAS_OSRS_P_X4					0x0C
#define CTRL_MEAS_OSRS_P_X8					0x10
#define CTRL_MEAS_OSRS_P_X16				0x1C
// mode
#define CTRL_MEAS_MODE_SLEEP				0x00
#define CTRL_MEAS_MODE_FORCE				0x01
#define CTRL_MEAS_MODE_NORMAL				0x03
//
////////////////////////////////////////////////////////
// Register "config"
#define CTRL_CONFIG_REG						0xF5
// t_sb
#define CTRL_CONFIG_T_SB_0_5_MS				0x00
#define CTRL_CONFIG_T_SB_62_5_MS			0x20
#define CTRL_CONFIG_T_SB_125_MS				0x40
#define CTRL_CONFIG_T_SB_250_MS				0x60
#define CTRL_CONFIG_T_SB_500_MS				0x80
#define CTRL_CONFIG_T_SB_1000_MS			0xA0
#define CTRL_CONFIG_T_SB_2000_MS			0xC0
#define CTRL_CONFIG_T_SB_4000_MS			0xE0
// filter
#define CTRL_CONFIG_FILTER_OFF				0x00
#define CTRL_CONFIG_FILTER_X2				0x04
#define CTRL_CONFIG_FILTER_X4				0x08
#define CTRL_CONFIG_FILTER_X8				0x0C
#define CTRL_CONFIG_FILTER_X16				0x10
// spi3w_en
#define CTRL_CONFIG_SPI_3_EN				0x00
#define CTRL_CONFIG_SPI_3_DIS				0x01
//
////////////////////////////////////////////////////////

static void calculation_PTA(uint8_t* data, float* pressure, int32_t* temperature, float* altitude);
static int32_t calc_temperature(int32_t ADC_value, int32_t* t_fine_for_pressure);
static float calc_pressure(int64_t ADC_value, int64_t t_fine_for_pressure);
static uint32_t make_uint32(uint32_t xmsb, uint32_t msb, uint32_t lsb, uint32_t xlsb);

static uint32_t g_status = BMP280_DRIVER_NO_ERROR;
static int32_t dig_T[3] = {0};
static int32_t dig_P[9] = {0};

//
// EXTERNAL INTERFACE
//
void BMP280_initialize() {

	g_status = BMP280_DRIVER_ERROR;

	// Read chip id
	uint8_t reg_data = 0;
	if (I2C_read_byte(I2C_ADDRESS, WHO_I_AM_REG, &reg_data) == false)
		return;

	// Check chip id
	if (reg_data != CHIP_ID)
		return;

	// Reset device
	if (I2C_write_byte(I2C_ADDRESS, CTRL_RESET_REG, CTRL_RESET_RESET) == false)
		return;
	delay(100);

	// Read calibration data
	uint16_t cal_data[12] = {0};
	if (I2C_read_bytes(I2C_ADDRESS, CALIBRATION_REG, (uint8_t*)&cal_data[0], CALIBRATION_DATA_SIZE) == false)
		return;
	
	dig_T[0] = (uint16_t)cal_data[0];
	dig_T[1] = (int16_t)cal_data[1];
	dig_T[2] = (int16_t)cal_data[2];

	dig_P[0] = (uint16_t)cal_data[3];
	dig_P[1] = (int16_t)cal_data[4];
	dig_P[2] = (int16_t)cal_data[5];
	dig_P[3] = (int16_t)cal_data[6];
	dig_P[4] = (int16_t)cal_data[7];
	dig_P[5] = (int16_t)cal_data[8];
	dig_P[6] = (int16_t)cal_data[9];
	dig_P[7] = (int16_t)cal_data[10];
	dig_P[8] = (int16_t)cal_data[11];

	// Set oversampling
	reg_data = CTRL_MEAS_MODE_NORMAL | CTRL_MEAS_OSRS_P_X16 | CTRL_MEAS_OSRS_T_X2;
	if (I2C_write_byte(I2C_ADDRESS, CTRL_MEAS_REG, reg_data) == false)
		return;

	// Set configuration
	reg_data = CTRL_CONFIG_SPI_3_DIS | CTRL_CONFIG_FILTER_X16 | CTRL_CONFIG_T_SB_0_5_MS;
	if (I2C_write_byte(I2C_ADDRESS, CTRL_CONFIG_REG, reg_data) == false)
		return;

	g_status = BMP280_DRIVER_NO_ERROR;
}

bool BMP280_is_data_ready() {

	g_status = BMP280_DRIVER_ERROR;

	uint8_t data = 0;
	if (I2C_read_byte(I2C_ADDRESS, STATUS_REG, &data) == false)
		return false;

	g_status = BMP280_DRIVER_NO_ERROR;
	return (data & 0x08) == 0;
}

void BMP280_get_data(float* pressure, int32_t* temperature, float* altitude) {

	static bool is_start_communication = false;
	if (is_start_communication == false) { // Start communication

		I2C_set_internal_address_length(1);
		if (I2C_async_read_bytes(I2C_ADDRESS, 0xF7, 6) == false) {
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
			g_status = BMP280_DRIVER_NO_ERROR;
		}
		else if (status == I2C_DRIVER_ERROR) {
			g_status = BMP280_DRIVER_ERROR;
		}
		else if (status == I2C_DRIVER_BUSY) {
			g_status = BMP280_DRIVER_BUSY;
			return;
		}

		is_start_communication = false;
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
static void calculation_PTA(uint8_t* data, float* pressure, int32_t* temperature, float* altitude) {

	// Make values
	uint32_t ADC_pressure = make_uint32(0, data[0], data[1], data[2] >> 4);
	uint32_t ADC_temperature = make_uint32(0, data[3], data[4], data[5] >> 4);

	// Calculate pressure and temperature
	int32_t fine_for_pressure = 0;
	int32_t T = calc_temperature(ADC_temperature, &fine_for_pressure);
	float P = calc_pressure(ADC_pressure, fine_for_pressure);

	// Calculate altitude
	if (altitude != nullptr)
		*altitude = 44330.0 * (1.0 - pow(P / 100 / 1013.25 /* normal pressure */, 0.1903)) * 100.0;

	// Write result
	if (pressure != nullptr)
		*pressure = P;

	if (temperature != nullptr)
		*temperature = T;
}

static int32_t calc_temperature(int32_t ADC_value, int32_t* t_fine_for_pressure) {

	ADC_value >>= 4;

	int32_t temp_1 = (ADC_value >> 3) - (dig_T[0] << 1);
	temp_1 *= dig_T[1];
	temp_1 >>= 11;

	int32_t temp_2 = (ADC_value >> 4) - dig_T[0];

	int32_t temp_3 = (temp_2 * temp_2) >> 12;
	temp_3 *= dig_T[2];
	temp_3 >>= 14;

	*t_fine_for_pressure = temp_1 + temp_3;
	
	return ( (temp_1 + temp_3) * 5 + 128 ) >> 8;;
}

static float calc_pressure(int64_t ADC_value, int64_t t_fine_for_pressure) {

	ADC_value >>= 4;

	int64_t var1 = t_fine_for_pressure - 128000;
	int64_t var2 = var1 * var1 * (int64_t)dig_P[5];
	var2 += (var1 * (int64_t)dig_P[4]) << 17;
	var2 += (int64_t)dig_P[3] << 35;
	var1 = ((var1 * var1 * (int64_t)dig_P[2]) >> 8) + ((var1 * (int64_t)dig_P[1]) << 12);
	var1 = ((((int64_t)0x01) << 47) + var1) * ((int64_t)dig_P[0]) >> 33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}
	int64_t p = 1048576 - ADC_value;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P[8]) * (p >> 13) * ( p>> 13)) >> 25;
	var2 = (((int64_t)dig_P[7]) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P[6])<<4);
	return p / 256.0;
}

static uint32_t make_uint32(uint32_t xmsb, uint32_t msb, uint32_t lsb, uint32_t xlsb) {
	return (xmsb << 24) | (msb << 16) | (lsb << 8) | (xlsb << 0);
}