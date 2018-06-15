#include <Arduino.h>
#include "LIBRARY\I2C.h"
#include "LIBRARY\ADC.h"
#include "LED.h"
#include "GPI.h"
#include "communication.h"
#include "monitoring.h"
#include "configuration.h"
#include "fly_core.h"
#include "util.h"
#define FATAL_ERRORS_MASK	(TXRX::MAIN_STA_CONFIG_ERROR |				\
							 TXRX::MAIN_STA_COMMUNICATION_BREAK |		\
							 TXRX::MAIN_STA_WIRELESS_LOW_VOLTAGE |		\
							 TXRX::MAIN_STA_SENSORS_LOW_VOLTAGE)

extern "C" {
	static void initialize_MCU(void);
}
static void intialize_FW();
static void error_status_update();
static void make_state_packet();

static uint32_t g_status = TXRX::MAIN_STA_NO_ERROR;

int main() {

	initialize_MCU();
	Serial.begin(460800);	// DEBUG
	intialize_FW();

	while (true) {

		//
		// MAIN CORE PROCESS
		//

		// Recieve and send data
		communication_process();

		// Hardware monitoring process
		monitoring_process();

		// Update main core error status
		error_status_update();

		// Check main core status
		if (g_status & TXRX::MAIN_STA_FATAL_ERROR) {
			g_comm_cp.command = TXRX::FLY_MODE_WAIT;
		}


		//
		// FLY CORE PROCESS
		//
		fly_core_process(&g_comm_cp);


		//
		// CONSTRUCT STATE PACKET
		//
		make_state_packet();


		//
		// LED INDICATOR PROCESS
		//
		led_process(g_comm_sp.main_core_status, g_comm_sp.fly_core_status);
	}
	return 0;
}

//
// INTERNAL INTERFACE
//
extern "C" {

	void __libc_init_array(void);

	static void initialize_MCU(void)
	{
		// Disable Watch Dog Timer
		WDT->WDT_MR = WDT_MR_WDDIS;

		SystemInit();

		// Set Systick to 1ms interval, common to all SAM3 variants
		if (SysTick_Config(SystemCoreClock / 1000))
			while (true);

		// Initialize C library
		__libc_init_array();

		// Enable PIOA and PIOB clocks 
		REG_PMC_PCER0 = PMC_PCER0_PID11 | PMC_PCER0_PID12;
		while ((REG_PMC_PCSR0 & (PMC_PCER0_PID11 | PMC_PCER0_PID12)) == 0);

		// Enable PIOC and PIOD clocks 
		REG_PMC_PCER0 = PMC_PCER0_PID13 | PMC_PCER0_PID14;
		while ((REG_PMC_PCSR0 & (PMC_PCER0_PID13 | PMC_PCER0_PID14)) == 0);


		// Disable pull-up on every pin
		REG_PIOA_PUDR = 0xFFFFFFFF;
		REG_PIOB_PUDR = 0xFFFFFFFF;
		REG_PIOC_PUDR = 0xFFFFFFFF;
		REG_PIOD_PUDR = 0xFFFFFFFF;

		// Enable parallel access on PIO output data registers
		REG_PIOA_OWER = 0xFFFFFFFF;
		REG_PIOB_OWER = 0xFFFFFFFF;
		REG_PIOC_OWER = 0xFFFFFFFF;
		REG_PIOD_OWER = 0xFFFFFFFF;

		// Initialize Serial port UART pins
		PIO_Configure(
			g_APinDescription[PINS_UART].pPort,
			g_APinDescription[PINS_UART].ulPinType,
			g_APinDescription[PINS_UART].ulPin,
			g_APinDescription[PINS_UART].ulPinConfiguration);
		digitalWrite(0, HIGH); // Enable pullup for RX0

		delay(1);
	}
}

static void intialize_FW() {

	gpi_initialize();
	led_initialize();

	// Initialize I2C wire
	I2C_initialize(I2C_SPEED_400KHZ);

	// Initialize configuration
	if (gpi_is_low(GPI_CONFIGURATION_MODE_INPUT) == true) {
		led_configuration_mode_enable();
		configuration_enter_to_change_mode();
	}
	if (gpi_is_low(GPI_RESET_CONFIGURATION_INPUT) == true) {

		if (configuration_reset() == false) {
			SET_BIT(g_status, TXRX::MAIN_STA_CONFIG_ERROR);
		}
	}
	if (configuration_load() == false) {
		SET_BIT(g_status, TXRX::MAIN_STA_CONFIG_ERROR);
	}

	communication_initialize();
	fly_core_initialize();
}

static void error_status_update() {

	// Clear all errors
	g_status = TXRX::MAIN_STA_NO_ERROR;

	// Check communication status
	uint32_t status = communication_get_status();
	if (IS_BIT_SET(status, COMMUNICATION_BREAK) == true)
		SET_BIT(g_status, TXRX::MAIN_STA_COMMUNICATION_BREAK);

	if (IS_BIT_SET(status, COMMUNICATION_DESYNC))
		SET_BIT(g_status, TXRX::MAIN_STA_COMMUNICATION_DESYNC);

	// Check monitoring status
	status = monitoring_get_status();
	if (IS_BIT_SET(status, MONITORING_MAIN_LOW_VOLTAGE) == true)
		SET_BIT(g_status, TXRX::MAIN_STA_MAIN_LOW_VOLTAGE);

	if (IS_BIT_SET(status, MONITORING_WIRELESS_LOW_VOLTAGE) == true)
		SET_BIT(g_status, TXRX::MAIN_STA_WIRELESS_LOW_VOLTAGE);

	if (IS_BIT_SET(status, MONITORING_SENSORS_LOW_VOLTAGE) == true)
		SET_BIT(g_status, TXRX::MAIN_STA_SENSORS_LOW_VOLTAGE);

	if (IS_BIT_SET(status, MONITORING_CAMERA_LOW_VOLTAGE) == true)
		SET_BIT(g_status, TXRX::MAIN_STA_CAMERA_LOW_VOLTAGE);

	// Check fatal errors
	if (g_status & FATAL_ERRORS_MASK)
		SET_BIT(g_status, TXRX::MAIN_STA_FATAL_ERROR);
}

static void make_state_packet() {

	// Clear packet
	memset(&g_comm_sp, 0, sizeof(g_comm_sp));

	g_comm_sp.main_core_status = g_status;

	fly_core_make_state_data(&g_comm_sp);
	monitoring_make_state_data(&g_comm_sp);
}
