/*
 * UART.h
 *
 * Created: 29.06.2018 15:42:58
 *  Author: makarov-a
 */ 


#ifndef UART_H_
#define UART_H_

extern void UART_initialize();
extern void UART_write(uint8_t* data, uint32_t size);
extern void UART_print_debug_message(const char* msg);
extern void UART_print_unsigned_number(uint32_t number);
extern void UART_print_number(int32_t number);

#endif /* UART_H_ */