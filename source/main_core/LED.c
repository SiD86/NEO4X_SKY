#include <stdbool.h>
#include "sam3x8e.h"
#include "systimer.h"
#include "fly_protocol.h"
#include "configuration.h"
#include "LED.h"
#include "utils.h"

#define MAIN_HW_FATAL_ERRORS_MASK	(FP_MAIN_STA_WIRELESS_LOW_VOLTAGE | FP_MAIN_STA_SENSORS_LOW_VOLTAGE)
#define FLY_HW_FATAL_ERRORS_MASK	(FP_FLY_STA_MPU6050_ERROR)

#define MAIN_HW_WARN_ERRORS_MASK	(FP_MAIN_STA_MAIN_LOW_VOLTAGE | FP_MAIN_STA_CAMERA_LOW_VOLTAGE)
#define FLY_HW_WARN_ERRORS_MASK		(FP_FLY_STA_BMP280_ERROR)

static void disable_all_LEDs(void);

void led_initialize(void) {

	// Blue LED
	REG_PIOC_PER  = PIO_PC21;
	REG_PIOC_OER  = PIO_PC21;
	REG_PIOC_CODR = PIO_PC21;

	// Red LED
	REG_PIOC_PER  = PIO_PC29;
	REG_PIOC_OER  = PIO_PC29;
	REG_PIOC_CODR = PIO_PC29;

	// Yellow LED
	REG_PIOD_PER  = PIO_PD7;
	REG_PIOD_OER  = PIO_PD7;
	REG_PIOD_CODR = PIO_PD7;

	// Green LED
	REG_PIOD_PER  = PIO_PD8;
	REG_PIOD_OER  = PIO_PD8;
	REG_PIOD_CODR = PIO_PD8;

	// Hull LEDS
	REG_PIOA_PER  = PIO_PA20;
	REG_PIOA_OER  = PIO_PA20;
	REG_PIOA_CODR = PIO_PA20;

	// Blink blue LED
	disable_all_LEDs();
	REG_PIOC_SODR = PIO_PC21;
	delay(100);

	// Blink red LED
	disable_all_LEDs();
	REG_PIOC_SODR = PIO_PC29;
	delay(100);

	// Blink yellow LED
	disable_all_LEDs();
	REG_PIOD_SODR = PIO_PD7;
	delay(50);

	// Blink green LED
	disable_all_LEDs();
	REG_PIOD_SODR = PIO_PD8;
	delay(50);

	disable_all_LEDs();
}

void led_configuration_mode_enable(void) {

	disable_all_LEDs();
	
	// Enable blue LED
	REG_PIOC_SODR = PIO_PC21;
}

void led_process(uint32_t main_core_status, uint32_t fly_core_status) {

	//
	// Process hull LEDs
	//
	static uint32_t prev_switch_time = 0;

	if (REG_PIOA_ODSR & PIO_PA20) {
		if (millis() - prev_switch_time > g_cfg.LED_enable_time) {
			REG_PIOA_CODR = PIO_PA20;	// Disable hull LEDs
			prev_switch_time = millis();
		}
	}
	else {
		if (millis() - prev_switch_time > g_cfg.LED_disable_time) {
			REG_PIOA_SODR = PIO_PA20;	// Enable hull LEDs
			prev_switch_time = millis();
		}
	}
	
	//
	// Process state LEDs
	//
	if (main_core_status & MAIN_HW_FATAL_ERRORS_MASK || fly_core_status & FLY_HW_FATAL_ERRORS_MASK) {

		if (REG_PIOC_ODSR & PIO_PC29)
			return;
		disable_all_LEDs();
		REG_PIOC_SODR = PIO_PC29;	// Enable red LED
	}
	else if (main_core_status & MAIN_HW_WARN_ERRORS_MASK || fly_core_status & FLY_HW_WARN_ERRORS_MASK) {

		if (REG_PIOD_ODSR & PIO_PD7)
			return;
		disable_all_LEDs();
		REG_PIOD_SODR = PIO_PD7;	// Enable yellow LED
	}
	else {

		if (REG_PIOD_ODSR & PIO_PD8)
			return;
		disable_all_LEDs();
		REG_PIOD_SODR = PIO_PD8; 	// Enable green LED
	}
}

static void disable_all_LEDs(void) {
	REG_PIOC_CODR = PIO_PC21;
	REG_PIOC_CODR = PIO_PC29;
	REG_PIOD_CODR = PIO_PD7;
	REG_PIOD_CODR = PIO_PD8;
}