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

        DriveTo(5000,0,40);
        DriveTo(5000,90,40);
        DriveTo(5000,-90,40);
        break;
    }

    return 0;

} //end main
