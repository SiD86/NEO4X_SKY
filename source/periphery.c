#include <stdbool.h>
#include "sam3x8e.h"
#include "periphery.h"

void periphery_initialize(void) {

	// Digital pins: 7, 6, 5, 4, 3
	REG_PIOC_PER  = PIO_PC23 | PIO_PC24 | PIO_PC25 | PIO_PC26 | PIO_PC28;
	REG_PIOC_ODR  = PIO_PC23 | PIO_PC24 | PIO_PC25 | PIO_PC26 | PIO_PC28;
	REG_PIOC_PUER = PIO_PC23 | PIO_PC24 | PIO_PC25 | PIO_PC26 | PIO_PC28;

	// Digital pin 2
	REG_PIOB_PER  = PIO_PB25;
	REG_PIOB_ODR  = PIO_PB25;
	REG_PIOB_PUER = PIO_PB25;
}

bool periphery_is_pin_low(uint32_t input) {

	switch (input) 
	{
	case PERIPHERY_ESC_CALIBRATION_INPUT:
		if ((REG_PIOC_PDSR & PIO_PC23) == 0)
			return true;
		break;

	case PERIPHERY_RESERVED1_INPUT:
		/*if ((REG_PIOC_PDSR & PIO_PC23) == 0)
			return true;*/
		break;

	case PERIPHERY_RESERVED2_INPUT:
		/*if ((REG_PIOC_PDSR & PIO_PC23) == 0)
			return true;*/
		break;

	case PERIPHERY_RESERVED3_INPUT:
		/*f ((REG_PIOC_PDSR & PIO_PC23) == 0)
			return true;*/
		break;

	case PERIPHERY_RESET_CONFIGURATION_INPUT:
		if ((REG_PIOC_PDSR & PIO_PC28) == 0)
			return true;
		break;

	case PERIPHERY_CONFIGURATION_MODE_INPUT:
		if ((REG_PIOB_PDSR & PIO_PB25) == 0)
			return true;
		break;
	}
	return false;
}