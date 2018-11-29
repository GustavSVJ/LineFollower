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


/****************************/
/*** UART2 Functions      ***/
/****************************/
void UART2_Putchar(char c);
char UART2_Getchar();
void UART2_Init(uint32_t baud);
void UART2_Putstr(char str[]);

#endif /* UART_H*/
