#include <stdio.h>
#include <string.h>
#include "stm32f30x_conf.h"
#include "Uart.h"
#include "lcd.h"
#include "flash.h"
#include "30021_io.h"
#include "uart_cam.h"

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
    init_usb_uart(921600);
    uart1_init(921600);
    init_USART1interrupt();

    while(1){
//        uart1_putc('i');
        //Delay
        for (uint32_t i = 0; i < 0xfffff; i++);
    }

    return 0;
}
/*********************************************************************/
void USART1_IRQHandler(){

    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET){

        uart1_putc('i');
        uart1_putc(USART_ReceiveData(USART1));

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

