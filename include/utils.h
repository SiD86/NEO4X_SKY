#ifndef UTILS_H_
#define UTILS_H_

#define SET_BIT(_status, _bit)				((_status) |= (_bit))
#define CLEAR_BIT(_status, _bit)			((_status) &= ~(_bit))
#define IS_BIT_SET(_status, _bit)			(((_status) & (_bit)) == (_bit))
#define IS_BIT_CLEAR(_status, _bit)			(((_status) & (_bit)) == 0)

#define constrain(amt,low,high)		( (amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)) )

#endif /* UTILS_H_ */