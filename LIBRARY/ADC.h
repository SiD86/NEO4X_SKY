#ifndef __ADC_H__
#define __ADC_H__

/**************************************************************************
* @brief	Function for initialize ADC
**************************************************************************/
void ADC_intialize();

/**************************************************************************
* @brief	Function for start ADC conversion with PDC
**************************************************************************/
void ADC_start_conversion();

/**************************************************************************
* @brief	Function for check conversion status
* @retval	true - ADC conversion done, false - ADC conversion in progress
**************************************************************************/
bool ADC_is_convesrion_complite();

/**************************************************************************
* @brief	Function for get channel measurement
* @param	channel_index: ADC channel index
* @retval	Channel measurement
**************************************************************************/
uint16_t ADC_get_data(uint32_t channel_index);

#endif /* __ADC_H__ */