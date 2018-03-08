#include <Arduino.h>
#include <variant.h>
#include "additional_subsystem.h"
#include "configuration_subsystem.h"
#include "util.h"
#define MEAS_VOLTAGE_PIN					(A0)
#define MEAS_VOLTAGE_PERIOD					(1000)
#define MEAS_VOLTAGE_ALPHA					(0.001)

static void process_measurements_battery_voltage(uint32_t adc);
static void error_status_update();

// Current state
static uint32_t g_battery_voltage = 0;
static uint32_t g_status = ASS::NO_ERROR;
static uint16_t g_ADC_buffer[10] = { 0 };

//
// EXTERNAL INTERFACE
//
void ASS::initialize() {

	// Initialize periphery
	pinMode(MEAS_VOLTAGE_PIN, INPUT);

	// Initialize ADC channels
	//adc_set_resolution(ADC, ADC_);
	REG_ADC_MR &= ~(ADC_MR_LOWRES_BITS_10); // 10 bit resolution
	REG_ADC_CHER =  ADC_CHER_CH0 |  // Battery voltage channel
					ADC_CHER_CH1 |  // Reserved
					ADC_CHER_CH2 |  // Reserved
					ADC_CHER_CH3 |  // Reserved
					ADC_CHER_CH4 |  // Reserved
					ADC_CHER_CH5 |  // Reserved
					ADC_CHER_CH6 |  // Reserved
					ADC_CHER_CH7 |  // Reserved
					ADC_CHER_CH10 | // Reserved
					ADC_CHER_CH11;  // Reserved

	// Reset status
	g_status = ASS::NO_ERROR;
}

void ASS::process() {

	static uint16_t ADC_channels_data[10] = { 0 };

	if (REG_ADC_RCR == 0) {

		memcpy(ADC_channels_data, g_ADC_buffer, sizeof(g_ADC_buffer));

		// Initialize PDC channel
		REG_ADC_PTCR = ADC_PTCR_RXTDIS;
		REG_ADC_RPR = (uint32_t)g_ADC_buffer;
		REG_ADC_RCR = 10;
		REG_ADC_PTCR = ADC_PTCR_RXTEN;

		// Start conversion
		REG_ADC_CR = ADC_CR_START;
	}
	else {
		return; // Data not ready
	}
	

	process_measurements_battery_voltage(ADC_channels_data[7] & 0x0FFF);

	// Measurement battery temperature

	// Measurement ESC temperature

	// Measurement brushless motors temperature

	// Measurement vibration brushless motors

	error_status_update();
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
static void process_measurements_battery_voltage(uint32_t adc) {

	// Filtering
	static float prev_flt_value = adc;
	float flt_value = adc * MEAS_VOLTAGE_ALPHA + prev_flt_value * (1 - MEAS_VOLTAGE_ALPHA);
	prev_flt_value = flt_value;

	// Calculation battery voltage
	// 3.3 - Max GPIO voltage
	// 4095 - 12 bit ADC max value
	// 656 - Voltage divisor coefficient (6.56)
	// Output: (V * 100)
	g_battery_voltage = flt_value * (3.3 / 4095.0) * 656;
}

static void error_status_update() {

	// Check battery voltage
	if (g_battery_voltage < g_cfg.battery_low_voltage)
		SET_STATUS_BIT(g_status, ASS::BATTERY_LOW_VOLTAGE);
	else
		CLEAR_STATUS_BIT(g_status, ASS::BATTERY_LOW_VOLTAGE);
}