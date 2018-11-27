#include "stm32f30x_conf.h" // STM32 config
#include "Uart.h"
#include "lcd.h"
#include "flash.h"
#include <stdio.h>
#include <string.h>
#include "30021_io.h"
#include "uart_cam.h"

/*********************************************************************/
int main(void){
    init_usb_uart(921600);
    uart1_init(921600);
    init_USART1interrupt();

    while(1){
        //Delay
        for (uint32_t i = 0; i < 0xfffff; i++);
    }

    return 0;
}

void USART1_IRQHandler(){

    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET){

    }
}

