#ifndef PERIPHERY_H_
#define PERIPHERY_H_

#define PERIPHERY_ESC_CALIBRATION_INPUT				(0x0000)
#define PERIPHERY_RESERVED1_INPUT					(0x0001)
#define PERIPHERY_RESERVED2_INPUT					(0x0002)
#define PERIPHERY_RESERVED3_INPUT					(0x0003)
#define PERIPHERY_RESET_CONFIGURATION_INPUT			(0x0004)
#define PERIPHERY_CONFIGURATION_MODE_INPUT			(0x0005)

extern void periphery_initialize(void);
extern bool periphery_is_pin_low(uint32_t input);

#endif /* PERIPHERY_H_ */