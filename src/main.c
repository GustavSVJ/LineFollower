#include <stdio.h>
#include <string.h>

#include "stm32f30x_conf.h"
#include "stm32f30x.h"

#include "Uart.h"
#include "UcamFunction.h"

#include "MotorControl.h"
#include "PID.h"

#include <math.h>


int main(void){

    //setup USB comm
    USB_Init(19200);

    //init delay functions delay_ms and delay100us
    init_delay_timer();

    //init ucam uart fifo
    uart_fifo_init(&uart_fifo, sizeof(uart_data_buffer), uart_data_buffer);

    //init and activate ucam uart
    UART1_Init(921600);
    UART1_EnableInterrupt();

    //init and setup camera
    ucam_init();

    //declare image in memory
    uint8_t image[4800];

    char buffer[10];


    while(1){

        //get picture from camera
        ucam_get_picture(image);

        for (uint16_t i = 0; i < 4800; i++){
            sprintf(buffer, "%u\r\n", image[i]);
            USB_Putstr(buffer);
        }

        break;

        //IMAGE processing

        //Drive to location


    }

    return 0;

} //end main
