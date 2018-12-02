#ifndef _UART_H_
#define _UART_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x_conf.h"
#include <stdio.h>


/****************************/
/*** USB Serial Functions ***/
/****************************/
void USB_Putchar(char c);
char USB_Getchar();
void USB_Init(uint32_t baud);
void USB_Putstr(char str[]);

/****************************/
/*** UART1 Functions      ***/
/****************************/
void UART1_Putchar(char c);
char UART1_Getchar();
void UART1_Init(uint32_t baud);
void UART1_Putstr(char str[]);
void UART1_EnableInterrupt();
void UART1_send_bytes(uint8_t *data, uint8_t no_bytes);



/****************************/
/*** UART2 Functions      ***/
/****************************/
void UART2_Putchar(char c);
char UART2_Getchar();
void UART2_Init(uint32_t baud);
void UART2_Putstr(char str[]);



/****************************/
/*** UART2 FIFO      ***/
/****************************/

typedef struct	{
	uint8_t *data;
	int head;
	int tail;
	int fifo_size;
} uart_fifo_t;


#define FIFO_SUCCESS    0
#define FIFO_FULL       -1
#define FIFO_EMPTY      -2

void uart_fifo_init(uart_fifo_t *fifo_in, uint8_t fifo_length, uint8_t *data);

int uart_fifo_read(uart_fifo_t *fifo_in, uint8_t *data_out);

int uart_fifo_write(uart_fifo_t *fifo_in, uint8_t *data_in);

int uart_fifo_elements(uart_fifo_t* fifo_in);


uint8_t uart_data_buffer[25];
uart_fifo_t uart_fifo;


#endif /* UART_H*/
