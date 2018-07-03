#include <stdbool.h>
#include "sam3x8e.h"
#include "pdg.h"
#include "systimer.h"
#include "utils.h"
#define MIN_POWER_PULSE				(1000)
#define MAX_POWER_PULSE				(2000)

/*
	[41]     [35]
		\   /
		 [ ]
		/   \
	[39]     [37]
*/
#define PULSE_TO_TICKS(_width)		(_width)
#define SET_FR_MOTOR_PULSE(_width)	(REG_PWM_CDTYUPD0 = PULSE_TO_TICKS(_width))
#define SET_RR_MOTOR_PULSE(_width)	(REG_PWM_CDTYUPD1 = PULSE_TO_TICKS(_width))
#define SET_RL_MOTOR_PULSE(_width)	(REG_PWM_CDTYUPD2 = PULSE_TO_TICKS(_width))
#define SET_FL_MOTOR_PULSE(_width)	(REG_PWM_CDTYUPD3 = PULSE_TO_TICKS(_width))

#define GET_FR_MOTOR_PULSE()		(REG_PWM_CDTY0)
#define GET_RR_MOTOR_PULSE()		(REG_PWM_CDTY1)
#define GET_RL_MOTOR_PULSE()		(REG_PWM_CDTY2)
#define GET_FL_MOTOR_PULSE()		(REG_PWM_CDTY3)



//
// EXTERNAL INTERFACE
//
void pdg_initialize(uint32_t ESC_frequency) {
	
	// Configure calibration enable pin
	REG_PIOC_PER = PIO_PC23;
	REG_PIOC_ODR = PIO_PC23;
	REG_PIOC_PUER = PIO_PC23;
	delay(1);

	// Get started duty cycle
	bool is_calibration_mode = false;
	uint32_t pulse_width = MIN_POWER_PULSE;
	if ((REG_PIOC_PDSR & PIO_PC23) == 0) {
		is_calibration_mode = true;
		pulse_width = MAX_POWER_PULSE;
	}

	//
	// Configure PWM
	//

	// Enable PWM clock
	REG_PMC_PCER1 = PMC_PCER1_PID36;
	while ((REG_PMC_PCSR1 & PMC_PCER1_PID36) == 0);
	
	// Setup PWM clock devider (84Mhz / 84 = 1MHz)
	REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(84);

	// Calculation period (PWM clock 1MHz)
	uint32_t period = 1000000 / ESC_frequency; // ESC_frequency = 1Mhz / period

	REG_PWM_SCM |= PWM_SCM_SYNC0 | PWM_SCM_SYNC1 | PWM_SCM_SYNC2 | PWM_SCM_SYNC3;
	REG_PWM_SCM |= PWM_SCM_UPDM_MODE0;
	
	REG_PIOC_PDR = PIO_PDR_P3 | PIO_PDR_P5 | PIO_PDR_P7 | PIO_PDR_P9;
	REG_PIOC_ABSR |= PIO_ABSR_P3 | PIO_ABSR_P5 | PIO_ABSR_P7 | PIO_ABSR_P9;
	REG_PWM_CMR0 = PWM_CMR_CPRE_CLKA | PWM_CMR_CPOL;
	REG_PWM_CMR1 = PWM_CMR_CPRE_CLKA | PWM_CMR_CPOL;
	REG_PWM_CMR2 = PWM_CMR_CPRE_CLKA | PWM_CMR_CPOL;
	REG_PWM_CMR3 = PWM_CMR_CPRE_CLKA | PWM_CMR_CPOL;
	REG_PWM_CDTY0 = PULSE_TO_TICKS(pulse_width);
	REG_PWM_CDTY1 = PULSE_TO_TICKS(pulse_width);
	REG_PWM_CDTY2 = PULSE_TO_TICKS(pulse_width);
	REG_PWM_CDTY3 = PULSE_TO_TICKS(pulse_width);
	REG_PWM_CPRD0 = period;
	
	// Enable all sync PWM channels
	REG_PWM_ENA = PWM_ENA_CHID0;
	while ((REG_PWM_SR & (PWM_ENA_CHID0 | PWM_ENA_CHID1 | PWM_ENA_CHID2 | PWM_ENA_CHID3)) == 0);


	if (is_calibration_mode == true) {
		delay(4000); // Wait ESC handling MAX pulse width
	}

	pdg_stop();
	delay(2000); // Wait ESC initialize complete
}

void pdg_stop() {
	SET_FR_MOTOR_PULSE(MIN_POWER_PULSE);
	SET_RR_MOTOR_PULSE(MIN_POWER_PULSE);
	SET_RL_MOTOR_PULSE(MIN_POWER_PULSE);
	SET_FL_MOTOR_PULSE(MIN_POWER_PULSE);

	REG_PWM_SCUC = PWM_SCUC_UPDULOCK;
}

void pdg_set_power(int32_t* power) {

	// Convert [0; 1000] to [1000; 2000]
	power[0] += MIN_POWER_PULSE;
	power[1] += MIN_POWER_PULSE;
	power[2] += MIN_POWER_PULSE;
	power[3] += MIN_POWER_PULSE;
	
	// Check range
	power[0] = constrain(power[0], MIN_POWER_PULSE, MAX_POWER_PULSE);
	power[1] = constrain(power[1], MIN_POWER_PULSE, MAX_POWER_PULSE);
	power[2] = constrain(power[2], MIN_POWER_PULSE, MAX_POWER_PULSE);
	power[3] = constrain(power[3], MIN_POWER_PULSE, MAX_POWER_PULSE);

	// Update pulse width
	SET_FR_MOTOR_PULSE(power[0]);
	SET_RR_MOTOR_PULSE(power[1]);
	SET_RL_MOTOR_PULSE(power[2]);
	SET_FL_MOTOR_PULSE(power[3]);

	REG_PWM_SCUC = PWM_SCUC_UPDULOCK;
}

void pdg_get_power_in_persent(uint8_t* power) {
	// Convert [1000; 2000] to [0; 100]
	power[0] = (GET_FR_MOTOR_PULSE() - MIN_POWER_PULSE) / 10;
	power[1] = (GET_RR_MOTOR_PULSE() - MIN_POWER_PULSE) / 10;
	power[2] = (GET_RL_MOTOR_PULSE() - MIN_POWER_PULSE) / 10;
	power[3] = (GET_FL_MOTOR_PULSE() - MIN_POWER_PULSE) / 10;
}
