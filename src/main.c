#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f30x_conf.h"
#include "Uart.h"

#include "MotorControl.h"


/**************************CAMERA FUNCTIONS***************************/
/*
        COMMAND         ID      P1              P2              P3                      P4
        ------------------------------------------------------------------------------------------------
        INITIAL         0xAA01  0x00            Image format    RAW res                 JPEG res
        GET PIC         0xAA04  Pic. type       0x00            0x00                    0x00
        SNAPSHOT        0xAA05  Snapsh. type    Skip frame(Lb)  Skip frame(Hb)          0x00
        SET PACK. SIZE  0xAA06  0x08            Pac.size (Lb)   Pac.size (Hb)           0x00

        SET BAUD        0xAA07  1. div          2. div          0x00                    0x00
        RESET           0xAA08  Reset type      0x00            0x00                    0xXX
        DATA            0xAA0A  Data type       Length Byte 0   Length Byte 1           Length Byte 2
        SYNC            0xAA0D  0x00            0x00            0x00                    0x00

        ACK             0xAA0E  Command ID      ACK counter     0x00/Package ID Byte 0  0x00/Package ID Byte 1
        NAK             0xAA0F  0x00            NACK counter    Error Number            0x00
        Light           0xAA13  Freq. type      0x00            0x00                    0x00

        Contrast/
        Brightness/     0xAA14  Contrast(0-4)   Brightness(0-4) Exposure(0-4)           0x00
        exposure

        SLEEP           0xAA15  Timeout(0-255)  0x00            0x00                    0x00

*/
/*********************************************************************/



int main(void){

    USB_Init(921600);


    uart_fifo_init(&uart_fifo, sizeof(uart_data_buffer), uart_data_buffer);

    UART1_Init(921600);
    UART1_EnableInterrupt();

    InitializeMotors();

    move_t newMovement;


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
    }

    return 0;
}
/*********************************************************************/



