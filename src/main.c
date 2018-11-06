#include "stm32f30x_conf.h" // STM32 config
#include "Uart.h"
#include "lcd.h"

void UART1_init();

int main(void){




    while(1)
    {

    }

    return 0;
}

void UART1_init()
    {
        USART_InitTypeDef USART_initStructure;

        //Enable GPIO clock
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        //Enable USART clock
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

        //UART configuration
        USART_initStructure.USART_BaudRate              = 115200;
        USART_initStructure.USART_WordLength            = USART_WordLength_8b;
        USART_initStructure.USART_StopBits              = USART_StopBits_1;
        USART_initStructure.USART_Parity                = USART_Parity_No;
        USART_initStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;
        USART_initStructure.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;
        USART_Init(UART1, &USART_initStructure);
    }
