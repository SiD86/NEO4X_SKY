#ifndef __UTIL_H__
#define __UTIL_H__

#define SET_STATUS_BIT(_status, _bit)		((_status) |= (_bit))
#define CLEAR_STATUS_BIT(_status, _bit)		((_status) &= ~(_bit))
#define IS_BIT_SET(_status, _bit)			(((_status) & (_bit)) == (_bit))
#define IS_BIT_CLEAR(_status, _bit)			(((_status) & (_bit)) == 0)

#define SET_DEBUG_PIN_1						(REG_PIOB_SODR = PIO_SODR_P14)
#define SET_DEBUG_PIN_2						(REG_PIOC_SODR = PIO_SODR_P13)
#define SET_DEBUG_PIN_3						(REG_PIOC_SODR = PIO_SODR_P14)
#define SET_DEBUG_PIN_4						(REG_PIOD_SODR = PIO_SODR_P2)
#define SET_DEBUG_PIN_5						(REG_PIOA_SODR = PIO_SODR_P15)
#define SET_DEBUG_PIN_6						(REG_PIOA_SODR = PIO_SODR_P14)

#define CLR_DEBUG_PIN_1						(REG_PIOB_CODR = PIO_SODR_P14)
#define CLR_DEBUG_PIN_2						(REG_PIOC_CODR = PIO_SODR_P13)
#define CLR_DEBUG_PIN_3						(REG_PIOC_CODR = PIO_SODR_P14)
#define CLR_DEBUG_PIN_4						(REG_PIOD_CODR = PIO_SODR_P2)
#define CLR_DEBUG_PIN_5						(REG_PIOA_CODR = PIO_SODR_P15)
#define CLR_DEBUG_PIN_6						(REG_PIOA_CODR = PIO_SODR_P14)

#endif /* __UTIL_H__ */