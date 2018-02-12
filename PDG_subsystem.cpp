#include <Arduino.h>
#include "TXRX_PROTOCOL.h"
#include "PDG_subsystem.h"
#include "CONFIG.h"
#define MIN_PWM_DUTY					(1000)
#define MAX_PWM_DUTY					(2000)

#define CHECK_SR(CH)					(REG_PWM_SR & (CH))
#define PWM_FR_MOTOR_W					(REG_PWM_CDTYUPD0)
#define PWM_RR_MOTOR_W					(REG_PWM_CDTYUPD1)
#define PWM_RL_MOTOR_W					(REG_PWM_CDTYUPD2)
#define PWM_FL_MOTOR_W					(REG_PWM_CDTYUPD3)

#define PWM_FR_MOTOR_R					(REG_PWM_CDTY0)
#define PWM_RR_MOTOR_R					(REG_PWM_CDTY1)
#define PWM_RL_MOTOR_R					(REG_PWM_CDTY2)
#define PWM_FL_MOTOR_R					(REG_PWM_CDTY3)

static void calibration_ESC();


//
// EXTERNAL INTERFACE
//
void PDGSS::initialize(uint32_t ESC_frequency) {

	// Enable PWM clock
	REG_PMC_PCER1 = PMC_PCER1_PID36;
	while ((REG_PMC_PCSR1 & PMC_PCER1_PID36) == 0);
	
	// Setup PWM clock devider (2 MHz = 84Mhz / 42)
	REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(42);

	// Calculation period (PWM clock 2MHz)
	uint32_t period = 1000000 / ESC_frequency; // ESC_frequency = 2Mhz / (2 * period)
	
	REG_PIOC_PDR = PIO_PDR_P3;										// Disable IO, enable peripheral function
	REG_PIOC_ABSR |= PIO_ABSR_P3;									// Select B peripheral function
	REG_PWM_CMR0 = PWM_CMR_CPRE_CLKA | PWM_CMR_CALG | PWM_CMR_CPOL;	// Select CLKA as clock source, select center alignment, signal inverted
	REG_PWM_CPRD0 = period;											// Set period
	REG_PWM_CDTY0 = 0;												// Set pulse width 
	
	REG_PIOC_PDR = PIO_PDR_P5;										// Disable IO, enable peripheral function
	REG_PIOC_ABSR |= PIO_ABSR_P5;									// Select B peripheral function
	REG_PWM_CMR1 = PWM_CMR_CPRE_CLKA | PWM_CMR_CALG | PWM_CMR_CPOL;	// Select CLKA as clock source, select center alignment, signal inverted
	REG_PWM_CPRD1 = period;											// Set period
	REG_PWM_CDTY1 = 0;												// Set pulse width 
	
	REG_PIOC_PDR = PIO_PDR_P7;										// Disable IO, enable peripheral function
	REG_PIOC_ABSR |= PIO_ABSR_P7;									// Select B peripheral function
	REG_PWM_CMR2 = PWM_CMR_CPRE_CLKA | PWM_CMR_CALG | PWM_CMR_CPOL;	// Select CLKA as clock source, select center alignment, signal inverted
	REG_PWM_CPRD2 = period;											// Set period
	REG_PWM_CDTY2 = 0;												// Set pulse width 
	
	REG_PIOC_PDR = PIO_PDR_P9;										// Disable IO, enable peripheral function
	REG_PIOC_ABSR |= PIO_ABSR_P9;									// Select B peripheral function
	REG_PWM_CMR3 = PWM_CMR_CPRE_CLKA | PWM_CMR_CALG | PWM_CMR_CPOL;	// Select CLKA as clock source, select center alignment, signal inverted
	REG_PWM_CPRD3 = period;											// Set period
	REG_PWM_CDTY3 = 0;												// Set pulse width 
	
	// Enable PWM
	REG_PWM_ENA = PWM_ENA_CHID0 | PWM_ENA_CHID1 | PWM_ENA_CHID2 | PWM_ENA_CHID3;
	while ((REG_PWM_SR & (PWM_ENA_CHID0 | PWM_ENA_CHID1 | PWM_ENA_CHID2 | PWM_ENA_CHID3)) == 0);

	// Check calibration ESC pin
	pinMode(2, INPUT_PULLUP);
	delay(10);
	if (digitalRead(2) == LOW)
		calibration_ESC();
}

void PDGSS::stop() {
	PWM_FR_MOTOR_W = MIN_PWM_DUTY;
	PWM_RR_MOTOR_W = MIN_PWM_DUTY;
	PWM_RL_MOTOR_W = MIN_PWM_DUTY;
	PWM_FL_MOTOR_W = MIN_PWM_DUTY;
}

void PDGSS::set_power(int32_t* power) {

	// Convert [0; 1000] to [1000; 2000]
	power[0] += MIN_PWM_DUTY;
	power[1] += MIN_PWM_DUTY;
	power[2] += MIN_PWM_DUTY;
	power[3] += MIN_PWM_DUTY;
	
	// Check range
	power[0] = constrain(power[0], MIN_PWM_DUTY, MAX_PWM_DUTY);
	power[1] = constrain(power[1], MIN_PWM_DUTY, MAX_PWM_DUTY);
	power[2] = constrain(power[2], MIN_PWM_DUTY, MAX_PWM_DUTY);
	power[3] = constrain(power[3], MIN_PWM_DUTY, MAX_PWM_DUTY);

	// Update pulse width
	PWM_FR_MOTOR_W = power[0];
	PWM_RR_MOTOR_W = power[1];
	PWM_RL_MOTOR_W = power[2];
	PWM_FL_MOTOR_W = power[3];
}

void PDGSS::get_power_in_persent(uint8_t* power) {

	if (PWM_FR_MOTOR_R == 0 || PWM_RR_MOTOR_R == 0 || PWM_RL_MOTOR_R == 0 || PWM_FL_MOTOR_R == 0)
		return;
		
	power[0] = (PWM_FR_MOTOR_R - MIN_PWM_DUTY) / 10;
	power[1] = (PWM_RR_MOTOR_R - MIN_PWM_DUTY) / 10;
	power[2] = (PWM_RL_MOTOR_R - MIN_PWM_DUTY) / 10;
	power[3] = (PWM_FL_MOTOR_R - MIN_PWM_DUTY) / 10;
}


//
// INTERNAL INTERFACE
//
static void calibration_ESC() {

	// Set maximum pulse width
	PWM_FR_MOTOR_W = MAX_PWM_DUTY;
	PWM_RR_MOTOR_W = MAX_PWM_DUTY;
	PWM_RL_MOTOR_W = MAX_PWM_DUTY;
	PWM_FL_MOTOR_W = MAX_PWM_DUTY;

	// Wait motors
	delay(3000);

	// Set minimum pulse width
	PWM_FR_MOTOR_W = MIN_PWM_DUTY;
	PWM_RR_MOTOR_W = MIN_PWM_DUTY;
	PWM_RL_MOTOR_W = MIN_PWM_DUTY;
	PWM_FL_MOTOR_W = MIN_PWM_DUTY;

	// Wait motors
	delay(3000);
}