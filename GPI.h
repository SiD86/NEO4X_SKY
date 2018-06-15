#ifndef __GPI_H__
#define __GPI_H__

#define GPI_ESC_CALIBRATION_INPUT				(0x0000)
#define GPI_RESERVED1_INPUT						(0x0001)
#define GPI_RESERVED2_INPUT						(0x0002)
#define GPI_RESERVED3_INPUT						(0x0003)
#define GPI_RESET_CONFIGURATION_INPUT			(0x0004)
#define GPI_CONFIGURATION_MODE_INPUT			(0x0005)

void gpi_initialize();
bool gpi_is_low(uint32_t input);

#endif /* __GPI_H__ */