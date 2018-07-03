#ifndef __ADC_H__
#define __ADC_H__

// ADC factor for convert bins to voltage
#define ADC_FACTOR							(3.3 / 4095.0)

// ADC channels map
#define ADC_MAIN_POWER_SUPPLY_CHANNEL		(5)
#define ADC_WIRELESS_POWER_SUPPLY_CHANNEL	(4)
#define ADC_CAMERA_POWER_SUPPLY_CHANNEL		(3)
#define ADC_SENSORS_POWER_SUPPLY_CHANNEL	(2)
#define ADC_TEMPERATURE_ESC_1				(1)
#define ADC_TEMPERATURE_ESC_2				(0)
#define ADC_TEMPERATURE_ESC_3				(6)
#define ADC_TEMPERATURE_ESC_4				(7)
#define ADC_VIBRATION_CHANNEL				(8)

/**************************************************************************
* @brief	Function for initialize ADC
**************************************************************************/
void ADC_intialize(void);

/**************************************************************************
* @brief	Function for start ADC conversion with PDC
**************************************************************************/
void ADC_start_conversion(void);

/**************************************************************************
* @brief	Function for check conversion status
* @retval	true - ADC conversion done, false - ADC conversion in progress
**************************************************************************/
bool ADC_is_convesrion_complite(void);

/**************************************************************************
* @brief	Function for get channel measurement
* @param	channel_index: ADC channel
* @retval	Channel measurement
**************************************************************************/
uint16_t ADC_get_data(uint32_t channel);

#endif /* __ADC_H__ */