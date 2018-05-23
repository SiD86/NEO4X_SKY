#include <Arduino.h>
#include "TXRX_PROTOCOL.h"
#include "LED.h"
#include "util.h"

#define MAIN_HW_FATAL_ERRORS_MASK	(TXRX::MAIN_STA_WIRELESS_POWER_SUPPLY | TXRX::MAIN_STA_SENSORS_POWER_SUPPLY)
#define FLY_HW_FATAL_ERRORS_MASK	(TXRX::FLY_STA_MPU6050_ERROR)

#define MAIN_HW_WARN_ERRORS_MASK	(TXRX::MAIN_STA_MAIN_POWER_SUPPLY | TXRX::MAIN_STA_CAMERA_POWER_SUPPLY)
#define FLY_HW_WARN_ERRORS_MASK		(TXRX::FLY_STA_BMP280_ERROR)

static void disable_all_LEDs();

void LED_initialize() {

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

void LED_configuration_mode_enable() {

	disable_all_LEDs();
	
	// Enable blue LED
	REG_PIOC_SODR = PIO_PC21;
}

void LED_process(uint32_t main_core_status, uint32_t fly_core_status) {

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

static void disable_all_LEDs() {
	REG_PIOC_CODR = PIO_PC21;
	REG_PIOC_CODR = PIO_PC29;
	REG_PIOD_CODR = PIO_PD7;
	REG_PIOD_CODR = PIO_PD8;
}