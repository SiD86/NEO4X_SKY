#include <Arduino.h>
#include "LIBRARY\I2C.h"
#include "communication_subsystem.h"
#include "additional_subsystem.h"
#include "configuration_subsystem.h"
#include "fly_core.h"
#include "util.h"
#define FATAL_ERRORS_MASK			(TXRX::MAIN_CORE_STATUS_CONFIG_ERROR | TXRX::MAIN_CORE_STATUS_COMM_LOST)

extern "C" {
	static void initialize_MCU(void);
}
static void intialize_FW();
static void error_status_update();
static void make_state_packet();

static uint32_t g_status = TXRX::MAIN_CORE_STATUS_NO_ERROR;

int main() {

	initialize_MCU();

	Serial.begin(460800);	// DEBUG

	/*pinMode(53, OUTPUT); // PB14
	pinMode(50, OUTPUT); // PC13
	pinMode(49, OUTPUT); // PC14
	pinMode(27, OUTPUT); // PD2
	pinMode(24, OUTPUT); // PA15
	pinMode(23, OUTPUT); // PA14

	CLR_DEBUG_PIN_1;
	CLR_DEBUG_PIN_2;
	CLR_DEBUG_PIN_3;
	CLR_DEBUG_PIN_4;
	CLR_DEBUG_PIN_5;
	CLR_DEBUG_PIN_6;*/
	
	intialize_FW();

	while (true) {
		
		//
		// MAIN CORE PROCESS
		//

		// Recieve and send data
		CSS::asynchronous_process();

		// Additional subsystem process
		ASS::process();

		// Update error status
		error_status_update();


		//
		// FLY CORE PROCESS
		//

		// Make command for fly core
		uint32_t fly_core_command = FLY_CORE::INTERNAL_CMD_PROCESS;
		if (g_status & TXRX::MAIN_CORE_STATUS_FATAL_ERROR) {
			fly_core_command = FLY_CORE::INTERNAL_CMD_DISABLE;
		}

		// Process fly core
		FLY_CORE::process(fly_core_command, &g_cp);


		//
		// CONSTRUCT STATE PACKET
		//

		// Update state data
		make_state_packet();
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

	// Initialize configuration subsystem
	if (CONFIGSS::intialize() == false)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_CONFIG_ERROR);

	// Initialize communication subsystem
	CSS::initialize();

	// Initialize additional subsystem
	ASS::initialize();

	// Initialize fly core
	FLY_CORE::initialize();
}

static void error_status_update() {

	// Check communication subsystem status
	uint32_t status = CSS::get_status();
	if (IS_BIT_SET(status, CSS::CONNECTION_LOST) == true)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_COMM_LOST);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_COMM_LOST);

	if (status & CSS::DESYNC)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_COMM_DESYNC);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_COMM_DESYNC);


	// Check additional subsystem status
	status = ASS::get_status();
	if (IS_BIT_SET(status, ASS::BATTERY_LOW_VOLTAGE) == true)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_12V_LOW_VOLTAGE);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_12V_LOW_VOLTAGE);

	// Check fatal errors
	if (g_status & FATAL_ERRORS_MASK)
		SET_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_FATAL_ERROR);
	else
		CLEAR_STATUS_BIT(g_status, TXRX::MAIN_CORE_STATUS_FATAL_ERROR);
}

extern uint8_t MPU6050_get_FIFO_size_error_count;
extern uint8_t MPU6050_check_FIFO_size_error_count;
extern uint8_t MPU6050_get_data_error_count;
extern volatile uint8_t I2C_nack_count;
extern volatile uint8_t I2C_timeout_count;
extern uint32_t g_PID_OOR_diff;
extern uint32_t g_PID_I_OOR_diff;
extern uint32_t g_hardware_error_count;
extern uint32_t g_software_error_count;
extern uint32_t g_desync_count;

static void make_state_packet() {

	// Clear packet
	memset(&g_sp, 0, sizeof(g_sp));

	g_sp.main_core_status = g_status;

	FLY_CORE::make_state_data(&g_sp);
	ASS::make_state_data(&g_sp);

	// Debug info
	g_sp.MPU6050_get_FIFO_size_error_count = MPU6050_get_FIFO_size_error_count;
	g_sp.MPU6050_check_FIFO_size_error_count = MPU6050_get_FIFO_size_error_count;
	g_sp.MPU6050_get_data_error_count = MPU6050_get_data_error_count;

	g_sp.I2C_nack_count = I2C_nack_count;
	g_sp.I2C_timeout_count = I2C_timeout_count;

	g_sp.PID_OOR_diff = g_PID_OOR_diff;
	g_sp.PID_I_OOR_diff = g_PID_I_OOR_diff;
}
