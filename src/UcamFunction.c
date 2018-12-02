#include "stm32f30x.h"
#include "Uart.h"
#include "UcamFunction.h"



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


ucam_status ucam_init(void){

    ucam_status status = UCAM_SUCCESS;

    ucam_reset();

    status = ucam_sync();

    return status;
}






ucam_status ucam_sync(void){

    uint8_t sync_data[6] = {0xAA, 0x0D, 0x00, 0x00, 0x00, 0x00};    //sync package to/from cam
    uint8_t ack_data[6]  = {0xAA, 0x0E, 0x0D, 0xDC, 0x00, 0x00};    //ack package to/from cam

    ucam_status status = UCAM_SYNC_FAIL;                            //variables
    uint8_t sync_tries = 60;
    uint8_t fifo_data;

    while(status == UCAM_SYNC_FAIL && sync_tries != 0){             //sync process, max 60 tries

        while(uart_fifo_read(&uart_fifo, &fifo_data) != FIFO_EMPTY);//empty uart fifo if data is present
        UART1_send_bytes(sync_data, 6);                             //send sync command
        delay_ms(5 + (60 - sync_tries));                            //wait for response

        if(uart_fifo_elements(&uart_fifo) >= 12){                   //check for response

            ucam_status status_temp = UCAM_SUCCESS;                 //temp status

            for(int i = 0; i < sizeof(ack_data); i++){              //check ack package from cam

                uart_fifo_read(&uart_fifo, &fifo_data);             //read element from fifo

                if(i == 3){                                         //compare to expected values
                    ack_data[i] = fifo_data;
                }
                else{
                    if(fifo_data != ack_data[i])
                        status_temp = UCAM_ACK_FAIL;                //change temp status if wrong data
                }
            }

            for(int i = 0; i < sizeof(sync_data); i++){             //check sync package from cam

                uart_fifo_read(&uart_fifo, &fifo_data);             //read element from fifo

                if(fifo_data != sync_data[i]){                      //compare to expected values
                    status_temp = UCAM_ACK_FAIL;                    //change temp status if wrong data
                }

            }

            if(status_temp == UCAM_SUCCESS){                        //packages ok check

                UART1_send_bytes(ack_data, 6);                      //send ack for cam sync
                status = status_temp;                               //update function status
            }
        }
    }
    return status;  //return final status
} //end ucam_sync





// ************************************ //
//  Ucam3 hardware reset functions      //
// ------------------------------------ //

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

    //reset ucam3 with PC08, reset is active low
    GPIOC->ODR &= ~(0x0001 << 8);

    //wait for 200ms
    delay_ms(200);

    //activate camera again
    GPIOC->ODR |= (0x0001 << 8);
}




// ************************************ //
//  Software delay functions            //
// ------------------------------------ //

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


void SysTick_Handler(void){
    //check if delay timer is set, and count down if it is
    if(delay_timer > 0){
        delay_timer--;
    }
}
