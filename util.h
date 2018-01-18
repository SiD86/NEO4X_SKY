#ifndef __UTIL_H__
#define __UTIL_H__

#define SET_STATUS_BIT(status, bit)			((status) |= (bit))
#define CLEAR_STATUS_BIT(status, bit)		((status) &= ~(bit))
#define IS_BIT_SET(status, bit)				(((status) & (bit)) == (bit))
#define IS_BIT_CLEAR(status, bit)			(((status) & (bit)) == 0)

#endif /* __UTIL_H__ */