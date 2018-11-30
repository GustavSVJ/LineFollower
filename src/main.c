#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f30x_conf.h"
#include "Uart.h"

#include "MotorControl.h"

#include "UcamFunction.h"




int main(void){

    USB_Init(921600);


    uart_fifo_init(&uart_fifo, sizeof(uart_data_buffer), uart_data_buffer);

    UART1_Init(921600);
    UART1_EnableInterrupt();

    InitializeMotors();

    move_t newMovement;

    ucam_init();


    char buffer[50];
        sprintf(buffer, "Hello World!\r\n");
        USB_Putstr(buffer);

    while(1){
        for (uint32_t i = 0; i < 0xfffff; i++);

        if(uart_fifo_elements(&uart_fifo)>0){

            char temp;
            uart_fifo_read(&uart_fifo, &temp);

            sprintf(buffer, "input: %c \r\n",temp);
            USB_Putstr(buffer);

        }
        uint8_t tester[6] = {72, 69, 74, 60, 51, 31};
        UART1_send_bytes(tester, 6);
    }

    return 0;
}
/*********************************************************************/



