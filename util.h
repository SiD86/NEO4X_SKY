#ifndef __UTIL_H__
#define __UTIL_H__

#define SET_STATUS_BIT(_status, _bit)		((_status) |= (_bit))
#define CLEAR_STATUS_BIT(_status, _bit)		((_status) &= ~(_bit))
#define IS_BIT_SET(_status, _bit)			(((_status) & (_bit)) == (_bit))
#define IS_BIT_CLEAR(_status, _bit)			(((_status) & (_bit)) == 0)

#endif /* __UTIL_H__ */