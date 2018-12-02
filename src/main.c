#include <stdio.h>
#include <string.h>
#include "stm32f30x_conf.h"

#include "Uart.h"
#include "MotorControl.h"
#include "PID.h"

char uart1_ReceiveBuffer[100];
volatile char uart1_RxFlag = 0;

int main(void){
    USB_Init(921600);

    RegulatorSetRefs(40,40,0);
    RegulatorRun();
    uint16_t i = 0;

    char buffer[50];

    while(1){
        if (timer15_PIDFlag){
            timer15_PIDFlag = 0;


            if (rightMotorTotalPulses > 40000 || leftMotorTotalPulses > 40000){
                RegulatorSetRefs(0,0,360);
                break;
            }
            else if(rightMotorTotalPulses > 30000 || leftMotorTotalPulses > 30000){
                RegulatorSetRefs(40,40,270);
            }
            else if(rightMotorTotalPulses > 20000 || leftMotorTotalPulses > 20000){
                RegulatorSetRefs(40,40,180);
            }
            else if(rightMotorTotalPulses > 10000 || leftMotorTotalPulses > 10000){
                RegulatorSetRefs(40,40,90);
            }


            RegulatorUpdate();



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

