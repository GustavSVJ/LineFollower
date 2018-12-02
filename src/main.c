#include <stdio.h>
#include <string.h>

#include "stm32f30x_conf.h"
#include "stm32f30x.h"

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

    uart_fifo_init(&uart_fifo, sizeof(uart_data_buffer), uart_data_buffer);

    UART1_Init(921600);
    UART1_EnableInterrupt();

    uint8_t tester[6] = {72, 69, 74, 60, 51, 31};
    UART1_send_bytes(tester, 6);

    ucam_init();

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

} //end main
