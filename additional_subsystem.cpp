#include <Arduino.h>
#include "LIBRARY\ADC.h"
#include "additional_subsystem.h"
#include "configuration_subsystem.h"
#include "util.h"
#define BATTERY_VOLTAGE_CHANNEL_INDEX			(0)//(7)
#define ESC_TEMPERATURE_CHANNEL_INDEX			(1)
#define MOTOR_TEMPERATURE_CHANNEL_INDEX			(5)
#define VIBRATION_CHANNEL_INDEX					(9)

#define STATE_START_CONVERSION					(0x02)
#define STATE_WAIT_CONVERSION_COMPLITE			(0x03)
#define STATE_PROCESS_BATTERY_VOLTAGE			(0x04)
#define STATE_PROCESS_ESC_TEMPERATURE			(0x05)
#define STATE_PROCESS_MOTOR_TEMPERATURE			(0x06)
#define STATE_PROCESS_VIBRATION					(0x07)
#define STATE_ANALYSIS_DATA						(0x08)

static uint32_t process_battery_voltage(uint32_t adc);
static uint32_t process_ESC_temperature(uint32_t adc);
static uint32_t process_motor_temperature(uint32_t adc);
static uint32_t process_vibration_level(uint32_t adc);
static void error_status_update();

// Measurements
static uint32_t g_battery_voltage = 0;
static uint32_t g_ESC_temperature[4] = { 0 };
static uint32_t g_motor_temperature[4] = { 0 };
static uint32_t g_vibration_level = 0;

// Current state
static uint32_t g_state = STATE_START_CONVERSION;
static uint32_t g_status = ASS::NO_ERROR;


//
// EXTERNAL INTERFACE
//
void ASS::initialize() {
	ADC_intialize();
	g_state = STATE_START_CONVERSION;
	g_status = ASS::NO_ERROR;
}

void ASS::process() {

	static uint32_t current_element = 0;

	uint16_t ADC_data = 0;

	switch (g_state)
	{
	case STATE_START_CONVERSION:
		ADC_start_conversion();
		g_state = STATE_WAIT_CONVERSION_COMPLITE;
		break;

	case STATE_WAIT_CONVERSION_COMPLITE:
		if (ADC_is_convesrion_complite() == true)
			g_state = STATE_PROCESS_BATTERY_VOLTAGE;
		break;

	case STATE_PROCESS_BATTERY_VOLTAGE:
		ADC_data = ADC_get_data(BATTERY_VOLTAGE_CHANNEL_INDEX);
		g_battery_voltage = process_battery_voltage(ADC_data);
		g_state = STATE_PROCESS_ESC_TEMPERATURE;
		break;

	case STATE_PROCESS_ESC_TEMPERATURE:
		ADC_data = ADC_get_data(ESC_TEMPERATURE_CHANNEL_INDEX + current_element);
		g_ESC_temperature[current_element] = process_ESC_temperature(ADC_data);
		++current_element;

		if (current_element >= 4) {
			current_element = 0;
			g_state = STATE_PROCESS_MOTOR_TEMPERATURE;
		}
		break;

	case STATE_PROCESS_MOTOR_TEMPERATURE:
		ADC_data = ADC_get_data(MOTOR_TEMPERATURE_CHANNEL_INDEX + current_element);
		g_motor_temperature[current_element] = process_motor_temperature(ADC_data);
		++current_element;

		if (current_element >= 4) {
			current_element = 0;
			g_state = STATE_PROCESS_VIBRATION;
		}
		break;

	case STATE_PROCESS_VIBRATION:
		ADC_data = ADC_get_data(VIBRATION_CHANNEL_INDEX);
		g_vibration_level = process_vibration_level(ADC_data);
		g_state = STATE_ANALYSIS_DATA;
		break;

	case STATE_ANALYSIS_DATA:
		error_status_update();
		g_state = STATE_START_CONVERSION;
		break;
	}
}

void ASS::make_state_data(TXRX::state_data_t* state_data) {

	state_data->battery_voltage = g_battery_voltage;
}

uint32_t ASS::get_status() {
	return g_status;
}


//
// INTERNAL INTERFACE
//
static uint32_t process_battery_voltage(uint32_t adc) {

	static const float alpha = 0.001;

	// Filtering
	static float prev_flt_value = adc;
	float flt_value = adc * alpha + prev_flt_value * (1 - alpha);
	prev_flt_value = flt_value;

	// Calculation battery voltage
	// 3.3 - Max GPIO voltage
	// 4095 - 12 bit ADC max value
	// 65.6 - Voltage divisor coefficient (6.56)
	// Output: (0.1V)
	//return flt_value * (3.3 / 4095.0) * 65.6;
	return adc;
}

static uint32_t process_ESC_temperature(uint32_t adc) {
	return adc;
}

static uint32_t process_motor_temperature(uint32_t adc) {
	return adc;
}

static uint32_t process_vibration_level(uint32_t adc) {
	return adc;
}

static void error_status_update() {

	/*Serial.print(g_battery_voltage);
	Serial.print(" ");
	for (int i = 0; i < 4; ++i) {
		Serial.print(g_ESC_temperature[i]);
		Serial.print(" ");
	}
	for (int i = 0; i < 4; ++i) {
		Serial.print(g_motor_temperature[i]);
		Serial.print(" ");
	}
	Serial.print(g_vibration_level);
	Serial.println("");*/

	// Check battery voltage
	if (g_battery_voltage < g_cfg.battery_low_voltage)
		SET_STATUS_BIT(g_status, ASS::BATTERY_LOW_VOLTAGE);
	else
		CLEAR_STATUS_BIT(g_status, ASS::BATTERY_LOW_VOLTAGE);

	// Check ESC temperature

	// Check motor temperature

	// Check vibration level
}