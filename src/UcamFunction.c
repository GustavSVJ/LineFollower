#include "stm32f30x.h"
#include "Uart.h"
#include "UcamFunction.h"
#include <stdlib.h>
#include <stdio.h>


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


void ucam_init(void){


    uint8_t cam_din[10];
    uint8_t status_cam = 0;

    //sync to cam
    while(status_cam == 0)
    {
        //initiate sync
        ucam_sync();

        //read bytes from ack
        for(int i = 0; i < 6; i++){
            uart_fifo_read(&uart_fifo, &cam_din[i]);
        }

        //decode bytes
        if( (cam_din[0] == 0xAA) &&  (cam_din[1] == 0x0E) && (cam_din[0] == 0x0D) ){
            status_cam = 1;
        }
    }


    while(uart_fifo_elements(&uart_fifo) < 6){
    }

    //read bytes from ack
    for(int i = 0; i < 6; i++){
        uart_fifo_read(&uart_fifo, &cam_din[i]);
    }


    status_cam = 1;

}






void ucam_sync(void){
    //sync routine

    uint8_t cam_data[6] = {0xAA, 0x0D, 0x00, 0x00, 0x00, 0x00};

    while(uart_fifo_elements(&uart_fifo) < 6){
        UART1_send_bytes(cam_data, 6);
        for(int i = 0; i < 100000; i++);
    }
}
 /*

void ucam_get_picture(){


}
*/



void ucam_reset_pin_setup(void){

    //setup PC08 as output for ucam reset pin
    //reset is active low

    RCC->AHBENR |= RCC_AHBPeriph_GPIOC;         //Enable clock for GPIO Port C

    GPIOC->MODER &= ~(0x00000003 << (8 * 2));   //clear MODER for PC08
    GPIOC->MODER |=  (0x00000001 << (8 * 2));   //set MODER for output

    GPIOC->PUPDR &= ~(0x00000003 << (0 * 2));   //clear PUPDR for PC08
    GPIOC->PUPDR |=  (0x00000001 << (0 * 2));   //set PUPDR for pull-up

    GPIOC->ODR |= (0x0001 << 8);                //set PCO8 as high
}


void ucam_reset(void){

    //reset ucam3 with PC08, active low


}


void delay_ms(uint32_t time_ms){

    //update SysTick delay timer
    delay_timer = time_ms;

    //wait until time_ms has passed
    while(delay_timer != 0);
}


void init_delay_ms(void){
    //setup systick to be used for ms delays
    SysTick_Config(64000000/1000);
}


void SysTick_Handler(void)
{
    //check if delay timer is set, and down count if so
    if(delay_timer > 0){
        delay_timer--;
    }
}
