#include <Arduino.h>
#include "LIBRARY\ADC.h"
#include "monitoring.h"
#include "configuration.h"
#include "util.h"

#define STATE_NO_INIT								(0x00)
#define STATE_START_CONVERSION						(0x01)
#define STATE_WAIT_CONVERSION_COMPLITE				(0x02)
#define STATE_PROCESS_ALL_POWER_SUPPLY_VOLTAGES		(0x03)
#define STATE_PROCESS_ESC_TEMPERATURE				(0x04)
#define STATE_PROCESS_HULL_VIBRATION				(0x05)
#define STATE_ANALYSIS_DATA							(0x06)

static void process_all_power_supply_voltages();
static void process_ESC_temperature();
static void process_hull_vibration();
static void error_status_update();
static void calc_flt_voltage(uint32_t adc, float VCC, float GND, float* flt_value);
static void calculate_temperature(uint32_t adc, float* flt_value);

// Measurements
static float g_main_power_supply_voltage = 0;
static float g_wireless_power_supply_voltage = 0;
static float g_camera_power_supply_voltage = 0;
static float g_sensors_power_supply_voltage = 0;
static float g_ESC_temperature[4] = { 0 };
static float g_hull_vibration = 0;

static uint32_t g_state = STATE_NO_INIT;
static uint32_t g_status = MONITORING_NO_ERROR;



//
// EXTERNAL INTERFACE
//
void monitoring_process() {

	switch (g_state)
	{
	case STATE_NO_INIT:
		ADC_intialize();
		g_state = STATE_START_CONVERSION;
		g_status = MONITORING_NO_ERROR;
		break;

	case STATE_START_CONVERSION:
		ADC_start_conversion();
		g_state = STATE_WAIT_CONVERSION_COMPLITE;
		break;

	case STATE_WAIT_CONVERSION_COMPLITE:
		if (ADC_is_convesrion_complite() == true)
			g_state = STATE_PROCESS_ALL_POWER_SUPPLY_VOLTAGES;
		break;

	case STATE_PROCESS_ALL_POWER_SUPPLY_VOLTAGES:
		process_all_power_supply_voltages();
		g_state = STATE_PROCESS_ESC_TEMPERATURE;
		break;

	case STATE_PROCESS_ESC_TEMPERATURE:
		process_ESC_temperature();
		g_state = STATE_PROCESS_HULL_VIBRATION;
		break;

	case STATE_PROCESS_HULL_VIBRATION:
		process_hull_vibration();
		g_state = STATE_ANALYSIS_DATA;
		break;

	case STATE_ANALYSIS_DATA:
		error_status_update();
		g_state = STATE_START_CONVERSION;
		break;
	}
}

void monitoring_make_state_data(TXRX::state_data_t* state_data) {

	// [0.1V]
	state_data->main_voltage = g_main_power_supply_voltage * 10.0;
	state_data->wireless_voltage = g_wireless_power_supply_voltage * 10.0;
	state_data->camera_voltage = g_camera_power_supply_voltage * 10.0;
	state_data->sensors_voltage = g_sensors_power_supply_voltage * 10.0;

	// [*C]
	state_data->ESC_temperature[0] = g_ESC_temperature[0];
	state_data->ESC_temperature[1] = g_ESC_temperature[1];
	state_data->ESC_temperature[2] = g_ESC_temperature[2];
	state_data->ESC_temperature[3] = g_ESC_temperature[3];

	// [?]
	state_data->hull_vibration = g_hull_vibration;
}

uint32_t monitoring_get_status() {
	return g_status;
}


//
// INTERNAL INTERFACE
//
static void process_all_power_supply_voltages() {

	//
	// MAIN POWER SUPPLY
	//

	// Calculate main power supply voltage
	uint32_t adc = ADC_get_data(ADC_MAIN_POWER_SUPPLY_CHANNEL);
	calc_flt_voltage(adc, 1000, 240, &g_main_power_supply_voltage);



	//
	// ADDITIONAL POWER SUPPLY
	//

	// Calculate wireless power supply voltage
	adc = ADC_get_data(ADC_WIRELESS_POWER_SUPPLY_CHANNEL);
	calc_flt_voltage(adc, 1000, 1000, &g_wireless_power_supply_voltage);

	// Calculate sensors power supply voltage
	adc = ADC_get_data(ADC_SENSORS_POWER_SUPPLY_CHANNEL);
	calc_flt_voltage(adc, 1000, 1000, &g_sensors_power_supply_voltage);

	// Calculate camera power supply voltage
	adc = ADC_get_data(ADC_CAMERA_POWER_SUPPLY_CHANNEL);
	calc_flt_voltage(adc, 1000, 1000, &g_camera_power_supply_voltage);
}

static void process_ESC_temperature() {

	static uint32_t current_ESC_index = 0;
	static uint32_t ADC_channels[] = { ADC_TEMPERATURE_ESC_1,
									   ADC_TEMPERATURE_ESC_2,
									   ADC_TEMPERATURE_ESC_3,
									   ADC_TEMPERATURE_ESC_4 };

	uint32_t adc = ADC_get_data(ADC_channels[current_ESC_index]);
	calculate_temperature(adc, &g_ESC_temperature[current_ESC_index]);

	if (++current_ESC_index >= 4)
		current_ESC_index = 0;
}

static void process_hull_vibration() {

	const float alpha = 0.0005; // Filtering parameter
	const float convert_factor = 255.0 / 4095.0;

	// Calculate ADC
	uint32_t adc = ADC_get_data(ADC_VIBRATION_CHANNEL);
	adc = adc * (3.3 / g_sensors_power_supply_voltage);
	adc = adc * convert_factor; // Convert [0; 4095] -> [0; 255]

	// Filtering [flt = alpha * uflt + (1 - alpha) * flt]
	if (g_hull_vibration == 0)
		g_hull_vibration = adc;
	else
		g_hull_vibration = alpha * adc + (1.0 - alpha) * g_hull_vibration;
}

static void error_status_update() {

	/*Serial.print(g_ESC_temperature[0]);
	Serial.print(" ");
	Serial.print(g_ESC_temperature[1]);
	Serial.print(" ");
	Serial.print(g_ESC_temperature[2]);
	Serial.print(" ");
	Serial.print(g_ESC_temperature[3]);
	Serial.println(" ");*/

	// Reset all errors
	g_status = MONITORING_NO_ERROR;

	// Check main power supply voltage
	if (g_main_power_supply_voltage < 9.5)
		SET_BIT(g_status, MONITORING_MAIN_LOW_VOLTAGE);

	// Check wireless power supply voltage
	if (g_wireless_power_supply_voltage < 4.0)
		SET_BIT(g_status, MONITORING_WIRELESS_LOW_VOLTAGE);

	// Check sensors power supply voltage
	if (g_sensors_power_supply_voltage < 2.7)
		SET_BIT(g_status, MONITORING_SENSORS_LOW_VOLTAGE);

	// Check camera power supply voltage
	if (g_camera_power_supply_voltage < 4.0)
		SET_BIT(g_status, MONITORING_CAMERA_LOW_VOLTAGE);


	// Check ESC temperature

	// Check motor temperature

	// Check vibration level
}

static void calc_flt_voltage(uint32_t adc, float VCC, float GND, float* flt_value) {

	const float alpha = 0.001; // Filtering parameter

	float divider_factor = (VCC + GND) / GND;
	float voltage = (adc * ADC_FACTOR) * divider_factor;

	// Filtering [flt = alpha * uflt + (1 - alpha) * flt]
	if (*flt_value == 0)
		*flt_value = voltage;
	else
		*flt_value = alpha * voltage + (1.0 - alpha) * (*flt_value);
}

static void calculate_temperature(uint32_t adc, float* flt_value) {

	// Temperature calculation parameters
	const float B = 4222;
	const float T20 = 293.15;
	const float R20 = 2200;
	const float R2 = 1000;

	// Parameter of complementary filter
	const float alpha = 0.1;

	// Calculating voltage from resistance divider
	// 3.3 - ADC voltage reference
	// 4095 - 12bit ADC resolution max value
	float voltage = ADC_FACTOR * adc;
	if (voltage == 0)
		*flt_value = 0.0;

	// Calculating resistance of thermal resistor 
	float R1 = (3.3 * R2) / voltage - R2;

	// Calculating temperature
	float LN_div_B = log(R1 / R20) / B;
	float T = 1.0 / (LN_div_B + (1.0 / T20)) - 273.15;

	// Filtering [flt = alpha * uflt + (1 - alpha) * flt]
	if (*flt_value == 0)
		*flt_value = voltage;
	else
		*flt_value = alpha * T + (1.0 - alpha) * (*flt_value);
}
