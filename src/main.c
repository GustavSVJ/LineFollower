#include <stdio.h>
#include <string.h>

#include "stm32f30x_conf.h"
#include "stm32f30x.h"

#include "Uart.h"
#include "UcamFunction.h"

#include "MotorControl.h"
#include "PID.h"


int main(void){

    //setup USB comm
    USB_Init(921600);

    //init delay functions delay_ms and delay100us
    init_delay_timer();

    //init ucam uart fifo
    uart_fifo_init(&uart_fifo, sizeof(uart_data_buffer), uart_data_buffer);

    //init and activate ucam uart
    UART1_Init(921600);
    UART1_EnableInterrupt();

    //init and setup camera
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
