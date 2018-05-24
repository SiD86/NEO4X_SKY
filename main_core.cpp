#include <Arduino.h>
#include "LIBRARY\I2C.h"
#include "LIBRARY\ADC.h"
#include "LED.h"
#include "GPI.h"
#include "communication_subsystem.h"
#include "additional_subsystem.h"
#include "configuration_subsystem.h"
#include "fly_core.h"
#include "util.h"
#define FATAL_ERRORS_MASK	(TXRX::MAIN_STA_CONFIG_ERROR |			\
							 TXRX::MAIN_STA_COMMUNICATION_LOST |	\
							 TXRX::MAIN_STA_WIRELESS_POWER_SUPPLY | \
							 TXRX::MAIN_STA_SENSORS_POWER_SUPPLY)

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
		CSS::asynchronous_process();

		// Additional subsystem process
		ASS::process();

		// Update main core error status
		error_status_update();


		//
		// FLY CORE PROCESS
		//

		// Make command for fly core
		uint32_t fly_core_command = FLY_CORE::INTERNAL_CMD_PROCESS;
		if (g_status & TXRX::MAIN_STA_FATAL_ERROR) {
			fly_core_command = FLY_CORE::INTERNAL_CMD_DISABLE;
		}

		// Process fly core
		FLY_CORE::process(fly_core_command, &g_cp);



		//
		// CONSTRUCT STATE PACKET
		//

		// Update state data
		make_state_packet();


		//
		// LED INDICATOR PROCESS
		//
		LED_process(g_sp.main_core_status, g_sp.fly_core_status);
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

	// Initialize I2C wire
	I2C_initialize(I2C_SPEED_400KHZ);

	GPI_initialize();
	LED_initialize();

	// Initialize configuration
	if (GPI_is_low(GPI_CONFIGURATION_MODE_INPUT) == true) {
		LED_configuration_mode_enable();
		CONFIG_enter_to_configuration_mode();
	}
	if (GPI_is_low(GPI_RESET_CONFIGURATION_INPUT) == true) {
		if (CONFIG_reset_configuration() == false)
			SET_STATUS_BIT(g_status, TXRX::MAIN_STA_CONFIG_ERROR);
	}
	if (CONFIG_load_and_check_configuration() == false) {
		SET_STATUS_BIT(g_status, TXRX::MAIN_STA_CONFIG_ERROR);
	}

	// Initialize communication subsystem
	CSS::initialize();

	// Initialize fly core
	FLY_CORE::initialize();
}

static void error_status_update() {

	// Check communication subsystem status
	uint32_t status = CSS::get_status();
	if (IS_BIT_SET(status, CSS::CONNECTION_LOST) == true)
		SET_STATUS_BIT(g_status, TXRX::MAIN_STA_COMMUNICATION_LOST);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_STA_COMMUNICATION_LOST);

	if (status & CSS::DESYNC)
		SET_STATUS_BIT(g_status, TXRX::MAIN_STA_COMMUNICATION_DESYNC);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_STA_COMMUNICATION_DESYNC);


	// Check additional subsystem status
	status = ASS::get_status();
	if (IS_BIT_SET(status, ASS::MAIN_POWER_SUPPLY_LOW_VOLTAGE) == true)
		SET_STATUS_BIT(g_status, TXRX::MAIN_STA_MAIN_POWER_SUPPLY);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_STA_MAIN_POWER_SUPPLY);

	if (IS_BIT_SET(status, ASS::WIRELESS_POWER_SUPPLY_LOW_VOLTAGE) == true)
		SET_STATUS_BIT(g_status, TXRX::MAIN_STA_WIRELESS_POWER_SUPPLY);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_STA_WIRELESS_POWER_SUPPLY);

	if (IS_BIT_SET(status, ASS::CAMERA_POWER_SUPPLY_LOW_VOLTAGE) == true)
		SET_STATUS_BIT(g_status, TXRX::MAIN_STA_CAMERA_POWER_SUPPLY);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_STA_CAMERA_POWER_SUPPLY);

	if (IS_BIT_SET(status, ASS::SENSORS_POWER_SUPPLY_LOW_VOLTAGE) == true)
		SET_STATUS_BIT(g_status, TXRX::MAIN_STA_SENSORS_POWER_SUPPLY);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_STA_SENSORS_POWER_SUPPLY);


	// Check fatal errors
	if (g_status & FATAL_ERRORS_MASK)
		SET_STATUS_BIT(g_status, TXRX::MAIN_STA_FATAL_ERROR);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_STA_FATAL_ERROR);
}

static void make_state_packet() {

	// Clear packet
	memset(&g_sp, 0, sizeof(g_sp));

	g_sp.main_core_status = g_status;

	FLY_CORE::make_state_data(&g_sp);
	ASS::make_state_data(&g_sp);
}
