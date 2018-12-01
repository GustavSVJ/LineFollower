#include <stdio.h>
#include <string.h>
#include "stm32f30x_conf.h"

#include "Uart.h"
#include "MotorControl.h"
#include "PID.h"



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

char uart1_ReceiveBuffer[100];
volatile char uart1_RxFlag = 0;




int main(void){
    USB_Init(921600);

    RegulatorRun();

    while(1){
        if (timer15_PIDFlag){
            timer15_PIDFlag = 0;
            RegulatorUpdate(40,40);

        }

    }

    return 0;
}
/*********************************************************************/


void USART1_IRQHandler(){

    static uint16_t i = 0;

    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET){
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);

        char rx = USART_ReceiveData(USART1);

        if (rx != '\n'){
            uart1_ReceiveBuffer[i] = rx;
            i++;
        }
        else{
            uart1_ReceiveBuffer[i] = 0;
            i = 0;
            uart1_RxFlag = 1;
        }
    }
}

