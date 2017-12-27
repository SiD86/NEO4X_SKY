#pragma once
#define SET_STATUS_BIT(status, bit)			((status) |= (bit))
#define CLEAR_STATUS_BIT(status, bit)		((status) &= ~(bit))