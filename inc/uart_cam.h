#ifndef UART_CAM
#define UART_CAM

#include "Uart.h"
#include "stm32f30x_conf.h"

#define receiveBuffer
#define receiveFlag
#define camera_ready

void uart1_putc(uint8_t c);
uint8_t uart1_getc();
void uart1_init(uint32_t baud);
void uart1_putstr(uint8_t str[]);

void uart2_putc(uint8_t c);
uint8_t uart2_getc();
void uart2_init(uint32_t baud);
void uart2_putstr(uint8_t str[]);

void init_USART1interrupt();

#endif
