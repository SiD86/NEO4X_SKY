#ifndef __GPI_H__
#define __GPI_H__

const uint32_t GPI_ESC_CALIBRATION_INPUT		= 0x0000;
const uint32_t GPI_RESERVED1_INPUT				= 0x0001;
const uint32_t GPI_RESERVED2_INPUT				= 0x0002;
const uint32_t GPI_RESERVED3_INPUT				= 0x0003;
const uint32_t GPI_RESET_CONFIGURATION_INPUT	= 0x0004;
const uint32_t GPI_CONFIGURATION_MODE_INPUT		= 0x0005;

void GPI_initialize();
bool GPI_is_low(uint32_t input);

#endif /* __GPI_H__ */