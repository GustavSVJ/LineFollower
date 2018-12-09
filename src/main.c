#include <stdio.h>
#include <string.h>
#include <math.h>

#include "stm32f30x_conf.h"
#include "stm32f30x.h"

#include "Uart.h"
#include "UcamFunction.h"

#include "pathfinder.h"

#include "MotorControl.h"
#include "PID.h"

#include <math.h>


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

    //setup motor
    RegulatorRun();

    //declare image in memory
    uint8_t image[4800];

    DriveTo(50,0,40);
    delay_ms(5000);

    while(1){

        DriveTo(50,90,40);
        delay_ms(5000);

    /*

        //get picture from camera
        ucam_get_picture(image);

        //IMAGE processing
        path_return_struct path_return;
        pathfinder(image, &path_return);

        delay_ms(50);

        //drive to first operation
        if(path_return.no_operations > 0){
            //convert data
            float temp = path_return.dist1 * 100 / 2;
            uint16_t distance = (uint16_t)temp;
            DriveTo(distance,path_return.rotate1,30);
            delay_ms(1500);
        }



        if(path_return.no_operations > 1){
            float temp = path_return.dist2 * 100 / 2;
            uint16_t distance = (uint16_t)temp;
            DriveTo(distance,path_return.rotate2,30);
            delay_ms(1500);
        }



        if(path_return.no_operations > 0){
            int i = 100;
        }
*/


    }



    return 0;

} //end main
