#include <stdio.h>
#include <string.h>

#include "stm32f30x_conf.h"
#include "stm32f30x.h"

#include "Uart.h"
#include "UcamFunction.h"

#include "pathfinder.h"

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

    //declare image in memory
    uint8_t image[4800];

    while(1){

        //get picture from camera
        ucam_get_picture(image);

        //IMAGE processing
        path_return_struct path_return;
        if(pathfinder(image, &path_return) == PATH_SUCCESS){
            USB_Putstr("pathfinder success \n\r");
        }

        //Drive to location
        DriveTo(5000,0,40);

    }

    return 0;

} //end main
